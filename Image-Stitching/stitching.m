I11=imread('img1.jpg');
I21=imread('img2.jpg');
I31=imread('img3.jpg');
I41=imread('img4.jpg');

I1 = rgb2gray(imread('img1.jpg'));
I2 = rgb2gray(imread('img2.jpg'));
I3 = rgb2gray(imread('img3.jpg'));
I4 = rgb2gray(imread('img4.jpg'));

[x1,y1,v1]=harris(I11);
points11=[x1 y1];
points1 = cornerPoints(points11); 

[x2,y2,v2]=harris(I21);
points21=[x2 y2];
points2 = cornerPoints(points21);

[x3,y3,v3]=harris(I31);
points31=[x3 y3];
points3 = cornerPoints(points31);

[x4,y4,v4]=harris(I41);
points41=[x4 y4];
points4 = cornerPoints(points41);

[features1,valid_points1] = extractFeatures(I1,points1);
[features2,valid_points2] = extractFeatures(I2,points2);
[features3,valid_points3] = extractFeatures(I3,points3);
[features4,valid_points4] = extractFeatures(I4,points4);

tform0 = [ 1 0 0;
          0 1 0;
         0 0 1 ];

tform(1) = affine2d(tform0);

indexPairs = matchFeatures(features1,features2);
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

%figure; showMatchedFeatures(I1,I2,matchedPoints2,matchedPoints1);
[tform(2), inlierDistorted, inlierOriginal] = estimateGeometricTransform(...
    matchedPoints2, matchedPoints1, 'similarity');
figure;
showMatchedFeatures(I1,I2, inlierOriginal, inlierDistorted);
title('Matching points (inliers only)');
legend('ptsOriginal','ptsDistorted');

%%%%%%%%%
indexPairs2 = matchFeatures(features2,features3);
matchedPoints21 = valid_points2(indexPairs2(:,1),:);
matchedPoints22 = valid_points3(indexPairs2(:,2),:);

%figure; showMatchedFeatures(I2,I3,matchedPoints22,matchedPoints21);
[tform(3), inlierDistorted, inlierOriginal] = estimateGeometricTransform(...
    matchedPoints22, matchedPoints21, 'similarity');
figure;
showMatchedFeatures(I2,I3, inlierOriginal, inlierDistorted);
title('Matching points (inliers only)');
legend('ptsOriginal','ptsDistorted');
tform(3).T=tform(2).T*tform(3).T;

%%%%%%%%
indexPairs3 = matchFeatures(features3,features4);
matchedPoints31 = valid_points3(indexPairs3(:,1),:);
matchedPoints32 = valid_points4(indexPairs3(:,2),:);

%figure; showMatchedFeatures(I3,I4,matchedPoints32,matchedPoints31);
[tform(4), inlierDistorted, inlierOriginal] = estimateGeometricTransform(...
    matchedPoints32, matchedPoints31, 'similarity');
figure;
showMatchedFeatures(I3,I4, inlierOriginal, inlierDistorted);
title('Matching points (inliers only)');
legend('ptsOriginal','ptsDistorted');
tform(4).T=tform(3).T*tform(4).T;

%Initialize the panaroma
for i=1:numel(tform)
    [xlim(i,:), ylim(i,:)] = outputLimits(tform(i), [1 2368], [1 4208]);
end

% Find the minimum and maximum output limits
xMin = min([1; xlim(:)]);
xMax = max([2368; xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([4208; ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I11);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
   % Transform I into the panorama.
    warpedImage = imwarp(I11, tform(1), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, warpedImage(:,:,1));
    % Transform I into the panorama.
    warpedImage = imwarp(I21, tform(2), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, warpedImage(:,:,1));

    % Transform I into the panorama.
    warpedImage = imwarp(I31, tform(3), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, warpedImage(:,:,1));

    % Transform I into the panorama.
    warpedImage = imwarp(I41, tform(4), 'OutputView', panoramaView);

    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, warpedImage(:,:,1));

figure

imshow(panorama)
imwrite(panorama,'stitched_final.jpg')
