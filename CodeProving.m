clc
clear all
%Mascaras
L1_matrix = readmatrix('Lotes_2021.xlsx','Sheet','043','Range','B2:BD178');
%L2 = readmatrix('Lotes_2021.xlsx','Sheet','386 A','Range','B2:AZ104');
%L3 = readmatrix('Lotes_2021.xlsx','Sheet','386 B','Range','B2:BK154');
L2_matrix = readmatrix('Lotes_2021.xlsx','Sheet','387','Range','B2:BE198');
L3_matrix = readmatrix('Lotes_2021.xlsx','Sheet','389A','Range','B2:Y160');
L4_matrix = readmatrix('Lotes_2021.xlsx','Sheet','389 B','Range','B2:AL137');
L5_matrix = readmatrix('Lotes_2021.xlsx','Sheet','395','Range','B2:BS160');
L6_matrix = readmatrix('Lotes_2021.xlsx','Sheet','399A','Range','B2:AK135');
L7_matrix = readmatrix('Lotes_2021.xlsx','Sheet','399B','Range','B2:AI173');

L1_mask = Get_Mask(L1_matrix);
L2_mask = Get_Mask(L2_matrix);
L3_mask = Get_Mask(L3_matrix);
L4_mask = Get_Mask(L4_matrix);
L5_mask = Get_Mask(L5_matrix);
L6_mask = Get_Mask(L6_matrix);
L7_mask = Get_Mask(L7_matrix);

L1_mask = im2uint16(L1_mask/65535);
L2_mask = im2uint16(L2_mask/65535);
L3_mask = im2uint16(L3_mask/65535);
L4_mask = im2uint16(L4_mask/65535);
L5_mask = im2uint16(L5_mask/65535);
L6_mask = im2uint16(L6_mask/65535);
L7_mask = im2uint16(L7_mask/65535);
% L8_mask = im2uint8(L8);
% L9_mask = im2uint8(L9);

img="2023_16bits.tif";
[imgdata,R] = readgeoraster(img);
%Geolocalización de la imagen
XLimits=R.XWorldLimits;
YLimits=R.YWorldLimits;
imgsize=size(imgdata);
YSize=imgsize(1);
XSize=imgsize(2);

%Delimitación del terreno
%Lote 43
Points_Lote43 =  [511904.70 8345427.60;511903.20 8345074.50;512095.50 8345306.30;511970.70 8345070.60];
%Lote 389B Surprise
Points_Lote389A = [512881.80 8344964.30; 512567.40 8345015.60;512577.30 8345093.50; 512567.40 8345015.60];
%Lote 389B Celebration
Points_Lote389B = [512558.60 8345018.70; 512288.60 8345058.20; 512570.50 8345094.80; 512312.80 8345183.10];
%Lote 395S
Points_Lote395S = [512466.10 8344975.80;512181.80 8344902.70; 512241.50 8344667.10; 512521.40 8344737.10];
%Lote 399SA
Points_Lote399B =[511747.60 8344781.50;512053.00 8344747.10; 512004.30 8344628.20;  511744.10 8344668.70];
%Lote 399SB
Points_Lote399A = [512210.10 8344725.80;  512053.00 8344747.10;512004.30 8344628.20; 512243.30 8344591.80];
%Lote 387S
Points_Lote387S = [512886.40 8344803.90; 512529.20 8344795.00; 512846.80 8344612.20; 512846.80 8344612.20];
%Lote 386S
%Points_Lote386S = [512568.60 8344586.90; 512870.70 8344588.30; 512870.70 8344588.30; 512399.60 8344373.70];
AllPoints=cat(3, Points_Lote43,Points_Lote387S,Points_Lote389A, Points_Lote389B, Points_Lote395S, Points_Lote399A, Points_Lote399B);
%Conversión de las coordenadas geográficas UTM a las coordenas de la imagen
Scaled_Points = zeros(4,2,7);
for k=1:7
    for i=1:4
        [Scaled_Points(i,1,k), Scaled_Points(i,2,k)] = ScaledPlanarPixel(AllPoints(i, 1, k), AllPoints(i, 2,k),XSize,YSize,XLimits,YLimits);
    end
end
Angles_Lotes=zeros(1,7);

for n=1:7
    Angles_Lotes(:,n) = ChangeDirectionImage(AllPoints(2,2,n), AllPoints(1,2,n), AllPoints(2,1,n), AllPoints(1,1,n));
end
RotatedP=zeros(4,2,7);
for j=1:7
    for m = 1:4
        RotatedP(m,:,j) = RotatePoints(imgdata(:,:,1), Angles_Lotes(j), [Scaled_Points(m, 2,j); Scaled_Points(m, 1,j)]); %(y,x)
    end
end

MinMax_Lotes = zeros(2,2,7);
for m=1:7
    Xmin=min(RotatedP(:,1,m));
    Xmax=max(RotatedP(:,1,m));
    Ymin=min(RotatedP(:,2,m));
    Ymax=max(RotatedP(:,2,m));
    MinMax_Lotes(1,1,m)=Xmin;
    MinMax_Lotes(1,2,m)=Ymin;
    MinMax_Lotes(2,1,m)=Xmax;
    MinMax_Lotes(2,2,m)=Ymax;
end
%Declaracion Lotes Rotados

for i=1:8
    L1_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(1));
    L2_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(2));
    L3_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(3));
    L4_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(4));
    L5_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(5));
    L6_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(6));
    L7_rotated(:,:,i)=imrotate(imgdata(:,:,i), Angles_Lotes(7));

    L1_segmented(:,:,i)=L1_rotated(MinMax_Lotes(1,2,1):MinMax_Lotes(2,2,1),MinMax_Lotes(1,1,1):MinMax_Lotes(2,1,1),i);
    L2_segmented(:,:,i)=L2_rotated(MinMax_Lotes(1,2,2):MinMax_Lotes(2,2,2),MinMax_Lotes(1,1,2):MinMax_Lotes(2,1,2),i);
    L3_segmented(:,:,i)=L3_rotated(MinMax_Lotes(1,2,3):MinMax_Lotes(2,2,3),MinMax_Lotes(1,1,3):MinMax_Lotes(2,1,3),i);
    L4_segmented(:,:,i)=L4_rotated(MinMax_Lotes(1,2,4):MinMax_Lotes(2,2,4),MinMax_Lotes(1,1,4):MinMax_Lotes(2,1,4),i);
    L5_segmented(:,:,i)=L5_rotated(MinMax_Lotes(1,2,5):MinMax_Lotes(2,2,5),MinMax_Lotes(1,1,5):MinMax_Lotes(2,1,5),i);
    L6_segmented(:,:,i)=L6_rotated(MinMax_Lotes(1,2,6):MinMax_Lotes(2,2,6),MinMax_Lotes(1,1,6):MinMax_Lotes(2,1,6),i);
    L7_segmented(:,:,i)=L6_rotated(MinMax_Lotes(1,2,7):MinMax_Lotes(2,2,7),MinMax_Lotes(1,1,7):MinMax_Lotes(2,1,7),i);
    
    L1_mask = imresize(L1_mask,size(L1_segmented(:,:,1)));
    L2_mask = imresize(L2_mask,size(L2_segmented(:,:,1)));
    L3_mask = imresize(L3_mask,size(L3_segmented(:,:,1)));
    L4_mask = imresize(L4_mask,size(L4_segmented(:,:,1)));
    L5_mask = imresize(L5_mask,size(L5_segmented(:,:,1)));
    L6_mask = imresize(L6_mask,size(L6_segmented(:,:,1)));
    L7_mask = imresize(L7_mask,size(L7_segmented(:,:,1)));

    % L1_resized(:,:,i) = imresize(L1_segmented(:,:,i),size(L1_mask));
    % L2_resized(:,:,i) = imresize(L2_segmented(:,:,i),size(L2_mask));
    % L3_resized(:,:,i) = imresize(L3_segmented(:,:,i),size(L3_mask));
    % L4_resized(:,:,i) = imresize(L4_segmented(:,:,i),size(L4_mask));
    % L5_resized(:,:,i) = imresize(L5_segmented(:,:,i),size(L5_mask));
    % L6_resized(:,:,i) = imresize(L6_segmented(:,:,i),size(L6_mask));
    % L7_resized(:,:,i) = imresize(L7_segmented(:,:,i),size(L7_mask));

    % L1(:,:,i) = L1_resized(:,:,i).*L1_mask;
    % L2(:,:,i) = L2_resized(:,:,i).*L2_mask;
    % L3(:,:,i) = L3_resized(:,:,i).*L3_mask;
    % L4(:,:,i) = L4_resized(:,:,i).*L4_mask;
    % L5(:,:,i) = L5_resized(:,:,i).*L5_mask;
    % L6(:,:,i) = L6_resized(:,:,i).*L6_mask;
    % L7(:,:,i) = L7_resized(:,:,i).*L7_mask;

    L1(:,:,i) = L1_segmented(:,:,i).*L1_mask;
    L2(:,:,i) = L2_segmented(:,:,i).*L2_mask;
    L3(:,:,i) = L3_segmented(:,:,i).*L3_mask;
    L4(:,:,i) = L4_segmented(:,:,i).*L4_mask;
    L5(:,:,i) = L5_segmented(:,:,i).*L5_mask;
    L6(:,:,i) = L6_segmented(:,:,i).*L6_mask;
    L7(:,:,i) = L7_segmented(:,:,i).*L7_mask;
end

L1_img=ShowImage(L1);
L2_img=ShowImage(L2);
L3_img=ShowImage(L3);
L4_img=ShowImage(L4);
L5_img=ShowImage(L5);
L6_img=ShowImage(L6);
L7_img=ShowImage(L7);
All_img=ShowImage(imgdata);

imshow(L1_img)
figure
imshow(L2_img)
figure
imshow(L3_img)
figure
imshow(L4_img)
figure
imshow(L5_img)
figure
imshow(L6_img)
figure
imshow(L7_img)
figure
imshow(All_img)
function [PixelX,PixelY] = ScaledPlanarPixel(PlanarX,PlanarY,XSize,YSize,XLimits,YLimits)

mx=(XSize-1)/(XLimits(2)-XLimits(1));
bx=XSize-mx*XLimits(2);
PixelX = uint16(mx*PlanarX+bx);

my=(YSize-1)/(YLimits(2)-YLimits(1));
by=YSize-my*YLimits(2);
PixelY = YSize-uint16(my*PlanarY+by)+1;
end
function angle= ChangeDirectionImage(y2, y1, x2, x1)
slope=(y2-y1)/(x2-x1);
angle=90-rad2deg(atan(slope));
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

function resultado = Get_Mask(matriz)
% Obtener dimensiones de la matriz
[filas, columnas] = size(matriz);

% Inicializar una matriz de resultados con ceros
resultado = zeros(filas, columnas);

% Iterar sobre cada elemento de la matriz
for i = 1:filas
    for j = 1:columnas
        % Si el valor es NaN, asignar cero; de lo contrario, asignar 1
        if isnan(matriz(i, j))
            resultado(i, j) = 0;
        else
            resultado(i, j) = 1;
        end
    end
end
end
function ImgForShowing=ShowImage(imgdata)
    BlueLayer=imgdata(:,:,2);
    GreenLayer=imgdata(:,:,4);
    RedLayer=imgdata(:,:,6);
    
    BlueLayer8B= im2uint8(BlueLayer);
    GreenLayer8B= im2uint8(GreenLayer);
    RedLayer8B= im2uint8(RedLayer);
    
    Rcomp = imcomplement(BlueLayer8B);
    Gcomp = imcomplement(GreenLayer8B);
    Bcomp = imcomplement(RedLayer8B);
    
    RGB_img=cat(3, Rcomp, Gcomp, Bcomp);
    ImgGray = rgb2gray(RGB_img);
    ImgForShowing = imadjust(ImgGray);
end