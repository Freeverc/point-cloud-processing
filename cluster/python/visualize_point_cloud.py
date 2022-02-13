import open3d as o3d
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker
from matplotlib import gridspec
# Author ：Xiaoyan，wx : 13260117850
from matplotlib.ticker import MultipleLocator, FormatStrFormatter, FuncFormatter

# 用来实现倒着显示colorbar的函数
def fmt(x, pos):
    # a, b = '{:2.2e}'.format(x).split('e')
    # b = int(b)
    return r'${}$'.format(int(50 - x))


# 用来实现倒着显示colorbar,重载Normalize的没有用的类
class InverseNormal(mpl.colors.Normalize):
    def __init__(self, boundaries, ncolors, clip=False):
        self.clip = clip
        self.vmin = min(boundaries)
        self.vmax = max(boundaries)
        self.boundaries = np.asarray(boundaries)
        self.N = len(self.boundaries)
        self.Ncmap = ncolors
        if self.N - 1 == self.Ncmap:
            self._interp = False
        else:
            self._interp = True

    def __call__(self, value, clip=None):
        if clip is None:
            clip = self.clip

        xx, is_scalar = self.process_value(value)
        mask = np.ma.getmaskarray(xx)
        xx = np.atleast_1d(xx.filled(self.vmax + 1))
        if clip:
            np.clip(xx, self.vmin, self.vmax, out=xx)
            max_col = self.Ncmap - 1
        else:
            max_col = self.Ncmap
        iret = np.zeros(xx.shape, dtype=np.int16)
        for i, b in enumerate(self.boundaries):
            iret[xx >= b] = i
        if self._interp:
            scalefac = (self.Ncmap - 1) / (self.N - 2)
            iret = (iret * scalefac).astype(np.int16)
        iret[xx < self.vmin] = -1
        iret[xx >= self.vmax] = max_col
        ret = 255 - np.ma.array(iret, mask=mask)
        if is_scalar:
            ret = int(ret[0])  # assume python scalar
        return ret


# 用来正则化数据
def normalize(data):
    mx = max(data)
    mn = min(data)
    return [(float(i) - mn) / (mx - mn) for i in data]


# 点云文件转figure
def to_fig(file_path):
    # 读取点云
    point_cloud = o3d.io.read_point_cloud(file_path)
    # 平移并且缩放到合适的尺度，使得xy坐标都可以按照10的间隔显示
    center = point_cloud.get_center()
    bounding_box = point_cloud.get_axis_aligned_bounding_box()
    scale = 80.0 / max(bounding_box.max_bound - bounding_box.min_bound)
    point_cloud.translate(np.array([0, 0, center[2]]), relative=False)
    point_cloud.scale(scale, np.array([0, 0, center[2]]))
    bounding_box = point_cloud.get_axis_aligned_bounding_box()
    print(bounding_box.min_bound)
    print(bounding_box.max_bound)
    print(len(point_cloud.points))
    print(len(point_cloud.colors))
    points = np.asarray(point_cloud.points)

    # 转成np数据，用于画图
    print(np.shape(points))
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # 设置colorbar
    cmap = plt.cm.get_cmap('jet')
    colors = cmap(normalize(z))

    # 画图
    fig = plt.figure(dpi=300)
    plt.title('point cloud')
    plt.axis('off')
    ax0 = Axes3D(fig)
    ax0.patch.set_facecolor('white')
    # 背景颜色
    ax0.grid(c='white')
    # 画所有点
    ax0.scatter(x, y, z, c=colors, marker='.', s=2, linewidth=0, alpha=1, cmap='spectral')

    # 设置标签和范围
    # 把x, y轴的刻度间隔设置为10，并存在变量里
    xy_major_locator = MultipleLocator(10)
    ax = plt.gca()
    h = np.array(range(0, 51))
    ax0.xaxis.set_major_locator(xy_major_locator)
    ax0.yaxis.set_major_locator(xy_major_locator)
    ax0.set_zlim(-40, 0)
    ax0.set_xlabel(' X(m)')
    ax0.set_ylabel(' Y(m)')
    ax0.set_zlabel(' Z(m)')

    # colorbar倒着显示
    norm_z = mpl.colors.BoundaryNorm(h, cmap.N)
    fcb = fig.colorbar(mpl.cm.ScalarMappable(norm=norm_z, cmap=cmap),
                       ax=ax0, shrink=0.6, format=ticker.FuncFormatter(fmt))
    # colorbar标签
    labels = list(reversed([i for i in range(-5, 60, 5)]))
    fcb.set_ticks(labels)
    fcb.set_label('水深(m)', fontproperties="SimHei")

    # 存储
    plt.savefig(file_path.replace('ply', 'png'))
    # plt.show()


if __name__ == '__main__':
    file_path = r'./color.ply'
    to_fig(file_path)
