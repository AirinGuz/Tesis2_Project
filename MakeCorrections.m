wavelenght=[443; 490; 531; 565; 610; 665; 705; 865];
hcube = hypercube("2023_16bits.tif", wavelenght);
rgbImg = colorize(hcube,'Method','RGB');
figure
imagesc(rgbImg)
title('RGB Image of Data Cube')