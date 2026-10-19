%% Image Panorama Stitching
scriptDir = fileparts(mfilename('fullpath'));
buildingDir = fullfile(scriptDir, "..", "data", "building_dir2");
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
[y, x, ~] = harris(grayImage, 1000, 'tile', [2 2], 'disp');
points = cornerPoints([x, y]);
[features, validPoints] = extractFeatures(grayImage, points);

%% Register Image Pairs
for n = 2:numImages
    featuresPrev = features;
    validPtsPrev = validPoints;
    I = readimage(buildingScene, n);
    grayImage = rgb2gray(I);
    imageSize(n,:) = size(grayImage);
    [y, x, ~] = harris(grayImage, 1000, 'tile', [2 2], 'disp');
    points = cornerPoints([x, y]);
    [features, validPoints] = extractFeatures(grayImage, points);
    indexPairs = matchFeatures(features, featuresPrev, ...
        'Unique', true, 'MatchThreshold', 50, 'MaxRatio', 0.9);
    matchedPts     = validPoints(indexPairs(:,1));
    matchedPtsPrev = validPtsPrev(indexPairs(:,2));
    tforms(n) = estimateGeometricTransform( ...
        matchedPts, matchedPtsPrev, ...
        'similarity', ...
        'Confidence', 99.9, 'MaxNumTrials', 2000);
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
xMin = min([1; xlim(:)]);  xMax = max([max(imageSize(:,2)); xlim(:)]);
yMin = min([1; ylim(:)]);  yMax = max([max(imageSize(:,1)); ylim(:)]);
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


panoramaView = imref2d([height width], [xMin xMax], [yMin yMax]);
accumImage  = zeros(height, width, 3, 'double');
accumWeight = zeros(height, width, 'double');

for idx = 1:numImages
    I = im2double(readimage(buildingScene, idx));
    [h, w, ~] = size(I);

    % Linear horizontal blend only
    [gridX, ~] = meshgrid(linspace(0,1,w), linspace(0,1,h));
    weight = min(gridX, 1-gridX) * 2;

    warpedImage  = imwarp(I,      tforms(idx), 'OutputView', panoramaView);
    warpedWeight = imwarp(weight, tforms(idx), 'OutputView', panoramaView);

    accumImage  = accumImage  + bsxfun(@times, warpedImage, warpedWeight);
    accumWeight = accumWeight + warpedWeight;
end

accumWeight(accumWeight == 0) = 1;
panorama = uint8(255 * bsxfun(@rdivide, accumImage, accumWeight));

%% Tight crop
grayPano = rgb2gray(panorama);
maskPano = grayPano > 0;
rowSums  = sum(maskPano, 2);
colSums  = sum(maskPano, 1);
fullRows = find(rowSums == width);
fullCols = find(colSums == height);

if ~isempty(fullCols) && ~isempty(fullRows)
    panoramaCropped = panorama(min(fullRows):max(fullRows), ...
                               min(fullCols):max(fullCols), :);
else
    rows = any(maskPano, 2);
    cols = any(maskPano, 1);
    panoramaCropped = panorama(find(rows,1,'first'):find(rows,1,'last'), ...
                               find(cols,1,'first'):find(cols,1,'last'), :);
end

%% Display
figure; imshow(panoramaCropped); title('Panorama')