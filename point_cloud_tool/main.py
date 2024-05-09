import tkinter as tk
import threading
import os
from tkinter import filedialog, ttk, messagebox
import tkinter.font as tkFont
import open3d as o3d
import numpy as np
import time
import point_cloud

class PointCloudViewer:
    def __init__(self, master):
        self.master = master
        master.title("点云处理器")
        master.geometry("600x900")

        myfont = tkFont.Font(family='song ti', size=18, weight=tkFont.BOLD, slant=tkFont.ITALIC)

        self.label = tk.Label(master, text="   选择点云文件, 拼接和生成网格", font=('song ti', 16, tkFont.BOLD))
        self.label.pack(pady=10, anchor=tk.NW)

        self.select_button = tk.Button(master, text="选择点云", command=self.select_point_cloud, width=30, height=3, font=myfont)
        self.select_button.pack(pady=10, anchor=tk.NW)

        self.merge_button = tk.Button(master, text="点云拼接", command=self.merge_point_clouds, width=30, height=3, font=myfont)
        self.merge_button.pack(pady=10, anchor=tk.NW)

        self.merge_button = tk.Button(master, text="网格生成", command=self.generate_mesh, width=30, height=3, font=myfont)
        self.merge_button.pack(pady=10, anchor=tk.NW)

        self.export_button = tk.Button(master, text="导出点云和网格", command=self.export_merged_cloud, width=30, height=3, font=myfont)
        self.export_button.pack(pady=10, anchor=tk.NW)

        self.clear_button = tk.Button(master, text="清理数据", command=self.clear_point_cloud, width=30, height=3, font=myfont)
        self.clear_button.pack(pady=10, anchor=tk.NW)

        self.file_list = []

        self.tree = ttk.Treeview(master, columns=('文件路径'), show="headings", height=40)
        self.tree.column('文件路径', width=400, anchor='center')
        self.tree.heading('文件路径', text='文件路径')
        self.tree.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=10)
        self.tree.bind("<Double-1>", self.on_item_double_click)

        self.merged_cloud = None
        self.output_path = ""

    def select_point_cloud(self):
        file_paths = filedialog.askopenfilenames(filetypes=[("Point Cloud Files", "*.pcd *.ply")])
        for file_path in file_paths:
            if file_path and file_path not in self.file_list:
                self.file_list.append(file_path)
                self.tree.insert('', tk.END, values=(file_path,))
            self.output_path = os.path.dirname(file_path)
        info_str = "导入了 " + str(len(file_paths)) + " 个文件, " +  "双击文件显示点云"
        messagebox.showinfo("提示", info_str)

    def on_item_double_click(self, event):
        item = self.tree.selection()[0]
        file_path = self.tree.item(item, "values")[0]
        self.display_point_cloud(file_path)

    def display_point_cloud(self, file_path):
        point_cloud = o3d.io.read_point_cloud(file_path)
        o3d.visualization.draw_geometries([point_cloud])

    def merge_point_clouds(self):
        if len(self.file_list) < 2:
            messagebox.showinfo("提示", "请至少选择两个点云文件进行拼接")
            return

        clouds = [o3d.io.read_point_cloud(fp).voxel_down_sample(voxel_size=0.01) for fp in self.file_list]
        self.merged_cloud = clouds[0]
        # self.merged_cloud = point_cloud.merge_point_clouds(clouds)
        for cloud in clouds[1:]:
            icp_result = o3d.pipelines.registration.registration_icp(
                cloud, clouds[0], 10,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
                
            self.merged_cloud += cloud.transform(icp_result.transformation)
            print(icp_result.transformation)
            print("merged len : ", len(self.merged_cloud.points))

        o3d.visualization.draw_geometries([self.merged_cloud])
        # messagebox.showinfo("提示", "点云拼接完成")

    def generate_mesh(self):
        print("init merged len : ", len(self.merged_cloud.points))
        self.merged_cloud, idx = self.merged_cloud.remove_statistical_outlier(nb_neighbors=6, std_ratio=1.0)
        print("sor merged len : ", len(self.merged_cloud.points))
        self.merged_cloud = self.merged_cloud.voxel_down_sample(voxel_size=0.5)
        print("filtered merged len : ", len(self.merged_cloud.points))

        self.merged_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.merged_cloud, depth=12)[0]
        # mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=1000)
        self.mesh = self.mesh.filter_smooth_simple(number_of_iterations=1)
        # self.mesh.paint_uniform_color([1, 1, 0]) 

        self.mesh.compute_vertex_normals()
        normals = np.asarray(self.mesh.vertex_normals)
        colors = (normals + 1) / 2  # 将法线从[-1, 1]映射到[0, 1]，以便用作颜色
        self.mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([self.mesh], mesh_show_back_face=True)

    def export_merged_cloud(self):
        if self.merged_cloud is None:
            messagebox.showinfo("提示", "请先进行点云拼接")
            return
        o3d.io.write_point_cloud(self.output_path + "/merged_pointcloud.ply", self.merged_cloud)
        o3d.io.write_triangle_mesh(self.output_path + "/mesh.ply", self.mesh)
        print( "点云和网格导出完成")
        # messagebox.showinfo("提示", "点云导出完成")
        # file_path = filedialog.asksaveasfilename(filetypes=[("Point Cloud Files", "*.pcd *.ply")])
        # if file_path:
        #     # 创建并启动一个新的线程来执行保存文件的操作
        #     points = self.merged_cloud
        #     threading.Thread(target=save_point_cloud, args=(file_path, points)).start()

    def clear_point_cloud(self):
        self.file_list = []
        self.output_path = ""
        for item in self.tree.get_children():
            self.tree.delete(item)


def main():
    root = tk.Tk()
    app = PointCloudViewer(root)
    root.mainloop()

if __name__ == "__main__":
    main()
