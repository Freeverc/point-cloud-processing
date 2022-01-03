# Point cloud compression

## octree
- Based on pcl.
- High compression rate, slow, lossy compression
### comands:
./octree_compression.exe .\longdress_vox10_1051.ply  .\compressed_octree.ot
./octree_uncompression.exe .\compressed_octree.ot .\uncompressed_octree.ply

## zlib
- Based on zlib.
- High compression rate, fast and lossless compression
### comands:
./zlib_compression.exe .\longdress_vox10_1051.ply  .\compressed_zlib.ot
./zlib_uncompression.exe .\compressed_zlib.ot .\uncompressed_zlib.ply

## Draco 
- Refer to https://github.com/google/draco.git
- High compression rate, very fast and lossy compression
### comands:
compression :   .\draco_encoder.exe  -point_cloud -i .\longdress_vox10_1051.ply -o compressed_draco.drc
uncompression : .\draco_decoder.exe -i .\compressed_draco.drc -o uncompressed_draco.ply