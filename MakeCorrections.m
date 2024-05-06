wavelenght=[443; 490; 531; 565; 610; 665; 705; 865];
hcube = hypercube("20201013_143255_48_2277_3B_AnalyticMS_SR_8b_clip.tif", wavelenght);
rgbImg = colorize(hcube,'Method','RGB');
figure
imagesc(rgbImg)
title('RGB Image of Data Cube')