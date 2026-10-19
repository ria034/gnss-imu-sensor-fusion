%% Image Panorama Stitching
scriptDir = fileparts(mfilename('fullpath'));
buildingDir = fullfile(scriptDir, "..", "data", "building_dir_cinder_wall");
buildingScene = imageDatastore(buildingDir);
montage(buildingScene.Files)

%% Extract Features from First Image
I = readimage(buildingScene, 1);
grayImage = rgb2gray(I);
numImages = numel(buildingScene.Files);
clear tforms
tforms(numImages) = affine2d;
imageSize = zeros(numImages, 2);
imageSize(1,:) = size(grayImage);

[y, x, ~] = harris(grayImage, 3000, 'tile', [4 4], 'disp');
points = cornerPoints([x, y]);
[features, validPoints] = extractFeatures(grayImage, points);

%% Register Image Pairs
for n = 2:numImages
    featuresPrev = features;
    validPtsPrev = validPoints;

    I = readimage(buildingScene, n);
    grayImage = rgb2gray(I);
    imageSize(n,:) = size(grayImage);

    [y, x, ~] = harris(grayImage, 3000, 'tile', [4 4], 'disp');
    points = cornerPoints([x, y]);
    [features, validPoints] = extractFeatures(grayImage, points);

    indexPairs = matchFeatures(features, featuresPrev, ...
        'Unique', true, 'MaxRatio', 0.6, 'MatchThreshold', 10);

    fprintf('Image %d: %d matches found\n', n, size(indexPairs, 1));

    if size(indexPairs, 1) < 10
        warning('Too few matches for image %d, using previous transform', n);
        tforms(n) = tforms(n-1);
        continue
    end

    matchedPts     = validPoints(indexPairs(:,1));
    matchedPtsPrev = validPtsPrev(indexPairs(:,2));

    tforms(n) = estimateGeometricTransform( ...
        matchedPts, matchedPtsPrev, ...
        'affine', ...                 % changed from projective
        'Confidence', 90, ...
        'MaxNumTrials', 3000, ...
        'MaxDistance', 2);

    tforms(n).T = tforms(n-1).T * tforms(n).T;
end

%% Find Center Image
xlim = zeros(numel(tforms), 2);
ylim = zeros(numel(tforms), 2);
for idx = 1:numel(tforms)
    [xlim(idx,:), ylim(idx,:)] = outputLimits(tforms(idx), ...
        [1 imageSize(idx,2)], [1 imageSize(idx,1)]);
end
avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerImageIdx = idx(floor((numel(tforms)+1)/2));

%% Adjust Transforms Relative to Center Image
Tinv = invert(tforms(centerImageIdx));
for idx = 1:numel(tforms)
    tforms(idx).T = Tinv.T * tforms(idx).T;
end

%% Recompute Output Limits
for idx = 1:numel(tforms)
    [xlim(idx,:), ylim(idx,:)] = outputLimits(tforms(idx), ...
        [1 imageSize(idx,2)], [1 imageSize(idx,1)]);
end

%% Canvas Size
xMin = min([1; xlim(:)]);
xMax = max([max(imageSize(:,2)); xlim(:)]);
yMin = min([1; ylim(:)]);
yMax = max([max(imageSize(:,1)); ylim(:)]);
width  = round(xMax - xMin);
height = round(yMax - yMin);
maxWidth  = 6 * max(imageSize(:,2));
maxHeight = 6 * max(imageSize(:,1));
if width > maxWidth || height > maxHeight
    width  = min(width, maxWidth);
    height = min(height, maxHeight);
    xMin = (xMin+xMax)/2 - width/2;  xMax = xMin + width;
    yMin = (yMin+yMax)/2 - height/2; yMax = yMin + height;
end
fprintf('Canvas: %d x %d (%.1f MP)\n', width, height, width*height/1e6);
panorama = zeros([height width 3], 'uint8');

%% Build Panorama with linear blending
panoramaView = imref2d([height width], [xMin xMax], [yMin yMax]);
blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource', 'Input port');
for idx = 1:numImages
    I           = readimage(buildingScene, idx);
    warpedImage = imwarp(I, tforms(idx), 'OutputView', panoramaView);
    mask        = imwarp(true(size(I,1), size(I,2)), tforms(idx), 'OutputView', panoramaView);
    panorama    = step(blender, panorama, warpedImage, mask);
end

%% Crop black borders
grayPano = rgb2gray(panorama);
maskPano = grayPano > 0;
rows = any(maskPano, 2);
cols = any(maskPano, 1);
rmin = find(rows, 1, 'first');
rmax = find(rows, 1, 'last');
cmin = find(cols, 1, 'first');
cmax = find(cols, 1, 'last');
panoramaCropped = panorama(rmin:rmax, cmin:cmax, :);

%% Display
figure; imshow(panoramaCropped); title('Panorama')