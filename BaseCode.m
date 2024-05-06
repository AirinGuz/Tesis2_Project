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
Point1X=511904.70;
Point1Y=8345427.60;

Point2X=512095.50;
Point2Y=8345306.30;

Point3X=511970.70;
Point3Y=8345070.60;
fundo_rotated = imrotate(fundo,angulo);

%mapshow(imgdata(:,:,1),R);
%mapshow(img_contrast,R);
%Conversión de las coordenadas geográficas UTM a las coordenas de la imagen
[Planar_Pixel_Scaled_X1,Planar_Pixel_Scaled_Y1] = ScaledPlanarPixel(Point1X,Point1Y,XSize,YSize,XLimits,YLimits);
[Planar_Pixel_Scaled_X2,Planar_Pixel_Scaled_Y2] = ScaledPlanarPixel(Point2X,Point2Y,XSize,YSize,XLimits,YLimits);
[Planar_Pixel_Scaled_X3,Planar_Pixel_Scaled_Y3] = ScaledPlanarPixel(Point3X,Point3Y,XSize,YSize,XLimits,YLimits);

imagesc(imgdata(:,:,1))
%mapshow(imgdata(:,:,1),R)
%Hallar el mínimo y máximo para definir la imagen segmentada
xmin=min([Planar_Pixel_Scaled_X1 Planar_Pixel_Scaled_X2 Planar_Pixel_Scaled_X3]);
xmax=max([Planar_Pixel_Scaled_X1 Planar_Pixel_Scaled_X2 Planar_Pixel_Scaled_X3]);
ymin=min([Planar_Pixel_Scaled_Y1 Planar_Pixel_Scaled_Y2 Planar_Pixel_Scaled_Y3]);
ymax=max([Planar_Pixel_Scaled_Y1 Planar_Pixel_Scaled_Y2 Planar_Pixel_Scaled_Y3]);
%Segmentación de la imagen
fundo=img_contrast(ymin:ymax,xmin:xmax);
%Colocación de markets sobre la imagen para validar la georeferenciacion
pos = [Planar_Pixel_Scaled_X1 Planar_Pixel_Scaled_Y1; Planar_Pixel_Scaled_X2 Planar_Pixel_Scaled_Y2; Planar_Pixel_Scaled_X3,Planar_Pixel_Scaled_Y3];
color = {"red", "blue", "green"};
RGB = insertMarker(img_contrast,pos, '+','color',color,'size',10);

imshow(RGB)

figure
M43_2021 = readmatrix('RACIMOS LOTE 043.xlsx','Sheet','Sheet1','Range','B2:BD179');
%Binarización de la parcela
M43_2021_binarized=imbinarize(M43_2021);
%Kernel para suma de vecinos de alrdedor de un pixel
K = [1 1 1;
     1 0 1;
     1 1 1;];
[rows_matrix, colummns_matrix]=size(M43_2021);
sum_neighbours=conv2(M43_2021_binarized,K,'same');
for i=1:rows_matrix
    for j=1:colummns_matrix
        if(sum_neighbours(i, j)>1 && i~=1 && i~=rows_matrix && j~=1 && j~=colummns_matrix)
            M43_2021_binarized(i, j)=1;
        end
    end
end
size_parcel_excel=size(M43_2021);
%Redimensionamiento del fundo teniendo en cuenta las dimensiones de la matriz del excel
resized_parcel_img = imresize(fundo,[size_parcel_excel(1) size_parcel_excel(2)]);

%Obtención de la máscara
MaskParcel=M43_2021_binarized;
OnlyParcel=imoverlay(resized_parcel_img, ~MaskParcel);


size_fundo_img=size(fundo);

%Segmentación y redimensionamiento de los bandas
CoastalBlueLayerParcel=CoastalBlueLayer(ymin:ymax,xmin:xmax,:);
ResizedCoastalBlueLayer = imresize(CoastalBlueLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

BlueLayerParcel=BlueLayer(ymin:ymax,xmin:xmax,:);
ResizedBlueLayer = imresize(BlueLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

GreenILayerParcel=GreenILayer(ymin:ymax,xmin:xmax,:);
ResizedGreenILayer = imresize(GreenILayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

GreenLayerParcel=GreenLayer(ymin:ymax,xmin:xmax,:);
ResizedGreenLayer = imresize(GreenLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

YellowLayerParcel=YellowLayer(ymin:ymax,xmin:xmax,:);
ResizedYellowLayer = imresize(YellowLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

RedLayerParcel=RedLayer(ymin:ymax,xmin:xmax,:);
ResizedRedLayer = imresize(RedLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

RedEdgeLayerParcel=RedEdgeLayer(ymin:ymax,xmin:xmax,:);
ResizedRedEdgeLayer = imresize(RedEdgeLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);

NIRLayerParcel=NIRLayer(ymin:ymax,xmin:xmax,:);
ResizedNIRLayerParcel = imresize(NIRLayerParcel,[size_parcel_excel(1) size_parcel_excel(2)]);
%Visualización de los resultados
imagesc(M43_2021)
figure
imshow(resized_parcel_img)
axis on
figure
imshow(OnlyParcel)
%Función para cambiar de coordenadas geográficas UTM a coordenas de la imagen
function [PixelX,PixelY] = ScaledPlanarPixel(PlanarX,PlanarY,XSize,YSize,XLimits,YLimits)

    mx=(XSize-1)/(XLimits(2)-XLimits(1));
    bx=XSize-mx*XLimits(2);
    PixelX = uint16(mx*PlanarX+bx);

    my=(YSize-1)/(YLimits(2)-YLimits(1));
    by=YSize-my*YLimits(2);
    PixelY = YSize-uint16(my*PlanarY+by)+1;
end
%%
