clc;clear;
fid=fopen('Depth_82.raw', 'r');
img= reshape(fread(fid,'uint16'), [640, 480]);
maxPixel = max(img,[],'all');
minPixel = 920;
img = (img - minPixel) / (maxPixel - minPixel) * 255;
imwrite(img, 'gray.png');