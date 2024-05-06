%%
%Lectura de la imagen
[imgdata,R] = readgeoraster("2023_16bits.tif");
%Declaración de las bandas existentes en la imagen
CoastalBlueLayer=imgdata(:,:,1);
BlueLayer=imgdata(:,:,2);
GreenILayer=imgdata(:,:,3);
GreenLayer=imgdata(:,:,4);
YellowLayer=imgdata(:,:,5);
RedLayer=imgdata(:,:,6);
RedEdgeLayer=imgdata(:,:,7);
NIRLayer=imgdata(:,:,8);
%Recuperación de la imagen RGB para efectos de visualización
BlueLayer8B= im2uint8(BlueLayer);
GreenLayer8B= im2uint8(GreenLayer);
RedLayer8B= im2uint8(RedLayer);

Rcomp = imcomplement(BlueLayer8B);
Gcomp = imcomplement(GreenLayer8B);
Bcomp = imcomplement(RedLayer8B);

RGB_img=cat(3, Rcomp, Gcomp, Bcomp);
ImgGray = rgb2gray(RGB_img);
img_contrast = imadjust(ImgGray);
%Geolocalización de la imagen
XLimits=R.XWorldLimits;
YLimits=R.YWorldLimits;
imgsize=size(imgdata);
YSize=imgsize(1);
XSize=imgsize(2);
%Delimitación del terreno

Points = [512881.80 8344964.30; 512567.40 8345015.60;512577.30 8345093.50; 512567.40 8345015.60];
%mapshow(imgdata(:,:,1),R);
%mapshow(img_contrast,R);
%Conversión de las coordenadas geográficas UTM a las coordenas de la imagen
for i=1:4
    [Points_Scaled(i,1),Points_Scaled(i,2)] = ScaledPlanarPixel(Points(i,1),Points(i,2),XSize,YSize,XLimits,YLimits);
end
imagesc(imgdata(:,:,1))
%mapshow(imgdata(:,:,1),R)
%Hallar el mínimo y máximo para definir la imagen segmentada
xmin=min(Points_Scaled(:,1));
xmax=max(Points_Scaled(:,1));
ymin=min(Points_Scaled(:,2));
ymax=max(Points_Scaled(:,2));
%Segmentación de la imagen
fundo=img_contrast(ymin:ymax,xmin:xmax);
%Colocación de markets sobre la imagen para validar la georeferenciacion
pos = Points_Scaled;

m=(Points(2,2)-Points(1,2))/(Points(2,1)-Points(1,1));
angulo=90-rad2deg(atan(m));
color = {"red", "blue", "green","white"};
RGB = insertMarker(img_contrast,pos, '+','color',color,'size',10);

fundo_rotated = imrotate(fundo,angulo);
img_rotated=imrotate(img_contrast, angulo);
RGB_rotated = imrotate(img_contrast, angulo);

P = [232; 8]; %(Y, X)
RotatedP_specific=RotatePoints(img_contrast, angulo, P);
RotatedP=zeros(4,2)
for i = 1:4
    RotatedP(i,:) = RotatePoints(img_contrast, angulo, [pos(i, 2); pos(i, 1)]); %(y,x)
end


RGB_1 = insertMarker(img_rotated,RotatedP, '+','color',color,'size',10);

imshow(RGB)
figure
imshow(RGB_1)
%New_Points([1 2],:)=New_Points([2 1],:);

% figure('Name','Before Rotation','NumberTitle','off');
% imshow(img_contrast);
% hold on
% plot(P(2),P(1),'r+', 'LineWidth', 20);
% plot(ImCenterA(2),ImCenterA(1),'k+', 'LineWidth', 20);
% figure('Name','After Rotation','NumberTitle','off');
% imshow(RotatedIm);
% hold on
% plot(RotatedP(2),RotatedP(1),'r+', 'LineWidth', 20);
% plot(ImCenterB(2),ImCenterB(1),'k+', 'LineWidth', 20);

function [PixelX,PixelY] = ScaledPlanarPixel(PlanarX,PlanarY,XSize,YSize,XLimits,YLimits)
    mx=(XSize-1)/(XLimits(2)-XLimits(1));
    bx=XSize-mx*XLimits(2);
    PixelX = uint16(mx*PlanarX+bx);

    my=(YSize-1)/(YLimits(2)-YLimits(1));
    by=YSize-my*YLimits(2);
    PixelY = YSize-uint16(my*PlanarY+by)+1;
end

function RotatedP = RotatePoints(img, alpha, P)
    RotatedIm = imrotate(img, alpha);   % Rotación de la imagen principal (im)
    RotMatrix = [cosd(alpha), -sind(alpha); sind(alpha), cosd(alpha)]; 
    ImCenterA = (size(img) / 2)';         % Centro de la imagen principal
    ImCenterB = (size(RotatedIm) / 2)';  % Centro de la imagen transformada
    P = double(P);
    RotatedP = uint16(round(RotMatrix * (P - ImCenterA) + ImCenterB));
    RotatedP([1 2],:)=RotatedP([2 1],:);
    RotatedP=transpose(RotatedP);
end