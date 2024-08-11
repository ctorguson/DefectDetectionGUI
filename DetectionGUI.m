function GUI()
% Create figure
f = figure('Name', 'Point Cloud Window', 'NumberTitle', 'off');
ax = axes(f);
set(ax, 'ButtonDownFcn', @(src, event) axes_ButtonDownFcn(src, event, f));
    function axes_ButtonDownFcn(~, event, f)
        % Get the current point clicked
        currentPoint = event.IntersectionPoint(1:3);
        % Assuming f.UserData.pcData contains your point cloud
        % Find the closest point in your point cloud
        Mdl = KDTreeSearcher(f.UserData.pcData.Location);
        closestIndex = knnsearch(Mdl, currentPoint, 'K', 1);
        % Store the index of the closest point in UserData
        f.UserData.selectedPointIndex = closestIndex;
    end

% Dropdown Menu for Opening Clouds
openMenu = uimenu(f, 'Label', 'Open');
newFormats = {'Cloud File', 'Merge Multiple', 'Mesh File'};
for i = 1:length(newFormats)
    uimenu(openMenu, 'Label', newFormats{i}, 'Callback', @(src, event) open_cloud_callback(src, event, f, ax, newFormats{i}));
end

% Add Merge Segments as a separate option
uimenu(openMenu, 'Label', 'Merge Segments', 'Callback', @(src, event) merge_segments_callback(src, event, f, ax));

% Dropdown Menu for Saving Options
saveMenu = uimenu(f, 'Label', 'Save Cloud As');
f.UserData.saveAllMenu = uimenu(saveMenu, 'Label', 'Save All Points', 'Enable', 'on', 'Callback', @(src, event) save_ply_callback(src, event, f, ax, 'all'));
f.UserData.saveBrushedMenu = uimenu(saveMenu, 'Label', 'Save Brushed Points', 'Enable', 'on', 'Callback', @(src, event) save_ply_callback(src, event, f, ax, 'brushed'));

% Dropdown Menu for Additional Visualization Options
vizOptionsMenu = uimenu(f, 'Label', 'Visualization');
gridOption = uimenu(vizOptionsMenu, 'Label', 'Grid', 'Callback', @(src, event) grid_option_callback(src, event, ax));
depthOption = uimenu(vizOptionsMenu, 'Label', 'Depth', 'Callback', @(src, event) depth_option_callback(src, event, f, ax));
resetOption = uimenu(vizOptionsMenu, 'Label', 'Reset', 'Callback', @(src, event) reset_option_callback(src, event, f, ax));
defectOption = uimenu(vizOptionsMenu, 'Label', 'Defect','Callback', @(src, event) defect_option_callback(src, event, f, ax));
densifyOption = uimenu(vizOptionsMenu, 'Label', 'Densify','Callback', @(src, event) densify_option_callback(src, event, f, ax));
planeOption = uimenu(vizOptionsMenu, 'Label', 'Plane', 'Callback', @(src, event) plane_option_callback(src, event, f, ax));
meshOption = uimenu(vizOptionsMenu, 'Label', 'Mesh', 'Callback', @(src, event) mesh_option_callback(src, event, f, ax));
flattenParabolicOption = uimenu(vizOptionsMenu, 'Label', 'Flat', 'Callback', @(src, event) flatten_parabolic_callback(src, event, f, ax));
alignXYOption = uimenu(vizOptionsMenu, 'Label', 'Align XY', 'Callback', @(src, event) align_xy_callback(src, event, f, ax));
end

function flatten_parabolic_callback(~, ~, f, ax)
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    pcData = f.UserData.pcData.Location;
    colorData = f.UserData.pcData.Color; % Extract color data

    % Check if intensity data is available
    if isfield(f.UserData.pcData, 'Intensity')
        intensityData = f.UserData.pcData.Intensity;
    else
        intensityData = [];
    end

    % Normalize the data for better numerical stability
    pcDataMean = mean(pcData, 1);
    pcDataStd = std(pcData, 0, 1);
    normPcData = (pcData - pcDataMean) ./ pcDataStd;

    % Fit a parabolic surface of the form z = ax^2 + by^2 + c to the point cloud data
    X = [normPcData(:,1).^2, normPcData(:,2).^2, ones(size(normPcData,1),1)];
    [abc, R] = lscov(X, normPcData(:,3)); % Weighted least squares solution

    % Check if the matrix is rank deficient
    if rank(R) < size(R, 2)
        warning('Rank deficient, the fit may not be accurate.');
    end

    % Compute the z-values of the fitted parabolic surface
    fittedZ = abc(1)*normPcData(:,1).^2 + abc(2)*normPcData(:,2).^2 + abc(3);

    % Subtract the fitted parabolic surface from the original z-values to flatten the primary curvature
    flattenedZ = normPcData(:,3) - fittedZ;

    % Denormalize the flattened Z values
    denormFlattenedZ = flattenedZ * pcDataStd(3) + pcDataMean(3);

    % Create the new point cloud data with the flattened z-values
    flattenedPoints = [pcData(:,1:2), denormFlattenedZ];

    % Create a new point cloud
    if isempty(intensityData)
        flattenedPC = pointCloud(flattenedPoints, 'Color', colorData);
    else
        flattenedPC = pointCloud(flattenedPoints, 'Color', colorData, 'Intensity', intensityData);
    end

    % Visualize the new, flattened point cloud
    pcshow(flattenedPC, 'Parent', ax);
    hold on;
    axis(ax, 'equal');
    hold off;

    % Store the flattened point cloud in the UserData
    f.UserData.pcData = flattenedPC;
else
    errordlg('No point cloud data available for fitting a plane!', 'Error');
end
end

function align_xy_callback(~, ~, f, ax)
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    pcData = f.UserData.pcData.Location;
    colorData = f.UserData.pcData.Color; % Extract color data

    % Check if intensity data is available
    if isfield(f.UserData.pcData, 'Intensity')
        intensityData = f.UserData.pcData.Intensity;
    else
        intensityData = [];
    end

    % Perform PCA on the point cloud data
    [coeff, score, ~] = pca(pcData);

    % The principal components are the columns of 'coeff'
    % Align the first principal component with the x-axis and the second with the y-axis
    alignedData = score(:, 1:3) * coeff(:, 1:3)';

    % Center the aligned data around the origin
    alignedData = bsxfun(@minus, alignedData, mean(alignedData));

    % Create a new point cloud with the aligned data
    if isempty(intensityData)
        alignedPC = pointCloud(alignedData, 'Color', colorData);
    else
        alignedPC = pointCloud(alignedData, 'Color', colorData, 'Intensity', intensityData);
    end

    % Visualize the aligned point cloud
    pcshow(alignedPC, 'Parent', ax);
    hold on;
    axis(ax, 'equal');
    view(ax, [0, 90]); % Set the view to be directly above the XY plane
    hold off;

    % Store the aligned point cloud in the UserData
    f.UserData.pcData = alignedPC;
else
    errordlg('No point cloud data available for alignment!', 'Error');
end
end

function mesh_option_callback(~, ~, f, ax)
% Check if point cloud data exists
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    % Access point cloud data
    pcData = f.UserData.pcData;
    % Denoise the point cloud
    pcDenoised = pcdenoise(pcData);
    % Resample the point cloud to improve meshing
    gridSize = 0.1; % Adjust this value based on your point cloud size and distribution
    pcDownsampled = pcdownsample(pcDenoised, 'gridAverage', gridSize);
    pts = double(pcDownsampled.Location); % Points
    colorData = double(pcDownsampled.Color) / 255; % Color
    % Delaunay triangulation
    DT = delaunayTriangulation(pts);
    tri = DT.ConnectivityList;
    % Interpolate vertex colors
    vertexColors = interpolate_vertex_colors(colorData, tri, pts);
    % Plot the mesh
    trimesh(tri, pts(:, 1), pts(:, 2), pts(:, 3), ...
        'FaceVertexCData', vertexColors, 'FaceColor', 'interp', 'EdgeColor', 'none', ...
        'Parent', ax);
    axis(ax, 'equal');
    % Store the triangulation back into f.UserData if needed
    f.UserData.triData = DT;
    guidata(f, f.UserData);
else
    errordlg('No point cloud data available for meshing!', 'Error');
end
end
function vertexColors = interpolate_vertex_colors(colorData, tri, pts)
vertexColors = zeros(size(pts, 1), 3);
for i = 1:size(pts, 1)
    % Find the closest point in the original point cloud to the vertex
    [~, closestPointIdx] = min(sum((pts(i, :) - colorData).^2, 2));
    % Assign the color of the closest point to the vertex
    vertexColors(i, :) = colorData(closestPointIdx, :);
end
end

% Logic for "Grid" option
function grid_option_callback(~, ~, ax)
% Get the current point cloud data
if ~isfield(ax.Parent.UserData, 'pcData') || isempty(ax.Parent.UserData.pcData)
    errordlg('No point cloud data available!', 'Error');
    return;
end

pcData = ax.Parent.UserData.pcData.Location;

% Calculate the bounding box of the point cloud
xMin = min(pcData(:, 1));
xMax = max(pcData(:, 1));
yMin = min(pcData(:, 2));
yMax = max(pcData(:, 2));
zMin = min(pcData(:, 3));
zMax = max(pcData(:, 3));

% Define the number of grid squares (adjusted to get about a dozen)
numSquaresX = 4; % Dividing the X range into 4 parts
numSquaresY = 4; % Dividing the Y range into 4 parts
numSquaresZ = 3; % Dividing the Z range into 3 parts

% Calculate the spacing between grid lines
xSpacing = (xMax - xMin) / numSquaresX;
ySpacing = (yMax - yMin) / numSquaresY;
zSpacing = (zMax - zMin) / numSquaresZ;

% Calculate grid line positions
xLines = xMin:xSpacing:xMax;
yLines = yMin:ySpacing:yMax;
zLines = zMin:zSpacing:zMax;

% Draw vertical grid lines along the X direction
for x = xLines
    for z = zLines
        line(ax, [x x], [yMin yMax], [z z], 'Color', 'k', 'LineWidth', 1);
    end
end

% Draw horizontal grid lines along the Y direction
for y = yLines
    for z = zLines
        line(ax, [xMin xMax], [y y], [z z], 'Color', 'k', 'LineWidth', 1);
    end
end

% Draw depth grid lines along the Z direction
for z = zLines
    for x = xLines
        line(ax, [x x], [yMin yMax], [z z], 'Color', 'k', 'LineWidth', 1);
    end
    for y = yLines
        line(ax, [xMin xMax], [y y], [z z], 'Color', 'k', 'LineWidth', 1);
    end
end

% Bring the point cloud to the top layer
if isfield(ax.UserData, 'pointCloudHandle')
    uistack(ax.UserData.pointCloudHandle, 'top');
end
end

% Logic for "Depth" option
function depth_option_callback(~, ~, f, ax)
if ~isempty(f.UserData.pcData)
    xyz = f.UserData.pcData.Location;
    pc = pointCloud(xyz); % Only XYZ components
    pcshow(pc, 'Parent', ax);
end
end

% Logic for "Reset" option
function reset_option_callback(~, ~, f, ax)
if isfield(f.UserData, 'originalPcData') && ~isempty(f.UserData.originalPcData)
    % Display original point cloud data
    pcshow(f.UserData.originalPcData, 'Parent', ax);
    f.UserData.pcData = f.UserData.originalPcData;
    title(ax, '');
    axis(ax, 'equal');
else
    errordlg('No original data to reset to!', 'Error');
end
end

% Logic for "Densify" option
function densify_option_callback(~, ~, f, ax)
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    % Access the existing point cloud data
    pcData = f.UserData.pcData.Location;
    colorData = f.UserData.pcData.Color;
    x = pcData(:, 1);
    y = pcData(:, 2);
    z = pcData(:, 3);
    r = double(colorData(:, 1));
    g = double(colorData(:, 2));
    b = double(colorData(:, 3));
    % Grid points for interpolation
    num_grid_points = 1000;
    xi = linspace(min(x), max(x), num_grid_points);
    yi = linspace(min(y), max(y), num_grid_points);
    [X, Y] = meshgrid(xi, yi);
    % Perform the interpolation (densification)
    Z = griddata(x, y, z, X, Y);
    R = griddata(x, y, r, X, Y);
    G = griddata(x, y, g, X, Y);
    B = griddata(x, y, b, X, Y);
    % Convert the densified mesh back to a point cloud
    densifiedPcData = pointCloud([X(:), Y(:), Z(:)], 'Color', uint8([R(:), G(:), B(:)]));
    % Display the new densified point cloud
    pcshow(densifiedPcData, 'Parent', ax);
    % Enable 3D rotation
    rotate3d on;
    % Save the new densified data back into f.UserData
    f.UserData.pcData = densifiedPcData;
    guidata(f, f.UserData);
    f.UserData.saveAllMenu.Enable = 'on';
    f.UserData.saveBrushedMenu.Enable = 'on';
else
    errordlg('No point cloud data available for densifying!', 'Error');
end
end

function plane_option_callback(~, ~, f, ax)
% Check if point cloud data exists
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    pcData = f.UserData.pcData.Location;
    colorData = f.UserData.pcData.Color; % Extract color data
    % Check if intensity data is available
    if isfield(f.UserData.pcData, 'Intensity')
        intensityData = f.UserData.pcData.Intensity;
    else
        intensityData = [];
    end
    % Compute the centroid of the points
    centroid = mean(pcData, 1);
    % Center the data
    centeredData = pcData - centroid;
    % Compute the singular value decomposition
    [~, ~, V] = svd(centeredData, 'econ');
    % The normal vector of the plane is the last column of V
    normal = V(:, 3);
    % Find the rotation matrix to align the normal with Z-axis
    rotAxis = cross(normal, [0 0 1]); % rotation axis is perpendicular to both vectors
    rotAngle = acos(dot(normal, [0 0 1])); % angle between normal and Z-axis
    rotMatrix = vrrotvec2mat([rotAxis, rotAngle]); % rotation matrix
    % Rotate all points in the point cloud
    rotatedPoints = (rotMatrix * centeredData')';
    % Create the rotated point cloud with color and optionally intensity
    if isempty(intensityData)
        rotatedPC = pointCloud(rotatedPoints, 'Color', colorData);
    else
        rotatedPC = pointCloud(rotatedPoints, 'Color', colorData, 'Intensity', intensityData);
    end
    % Store the rotated point cloud in the UserData
    f.UserData.pcData = rotatedPC;
    % Visualize the rotated point cloud
    pcshow(rotatedPC, 'Parent', ax);
    hold on;
    axis(ax, 'equal');
    hold off;
else
    errordlg('No point cloud data available for fitting a plane!', 'Error');
end
end
function pc = readPCD(filename)
fid = fopen(filename, 'r');
% Skip header lines
headerLineCount = 11; % This is a general assumption; adjust based on your PCD file format.
for i = 1:headerLineCount
    line = fgets(fid);
    if contains(line, 'FIELDS x y z intensity')
        formatSpec = '%f %f %f %f'; % XYZI
    else
        formatSpec = '%f %f %f'; % Assume XYZ by default
    end
end
% Read data
data = fscanf(fid, formatSpec, [length(strfind(formatSpec, '%f')), inf]);
data = data'; % Transpose data to have M-by-(3 or 4) shape
fclose(fid);
if size(data, 2) == 4 % If data has intensity values
    xyzData = data(:, 1:3);
    intensityData = data(:, 4);
    pc = pointCloud(xyzData, 'Intensity', intensityData);
else % Assume just XYZ data
    pc = pointCloud(data);
    disp(size(data)); % Display the size of data for debugging
    pc = pointCloud(data);
end
end

function defect_option_callback(~, ~, f, ax)
if isfield(f.UserData, 'originalPcData') && ~isempty(f.UserData.originalPcData)
    highlightedPointCloud = copy(f.UserData.originalPcData); % Make a copy here
    % Define the number of neighboring points to select
    numNeighbors = floor(f.UserData.originalPcData.Count * (0.07 + rand() * 0.03));
    % Choose a random index from the point cloud
    randomIndex = randi(f.UserData.originalPcData.Count);
    randomPoint = f.UserData.originalPcData.Location(randomIndex, :);
    % Use knnsearch to get the nearest neighbors to the random point
    Mdl = KDTreeSearcher(f.UserData.originalPcData.Location);
    highlightedIndices = knnsearch(Mdl, randomPoint, 'K', numNeighbors);
    % Get the height values for highlighted points
    highlightedHeights = highlightedPointCloud.Location(highlightedIndices, 3);
    % Calculate color values based on the height values
    minHighlightHeight = min(highlightedHeights);
    maxHighlightHeight = max(highlightedHeights);
    normalizedHeights = (highlightedHeights - minHighlightHeight) / (maxHighlightHeight - minHighlightHeight);
    % Use the jet colormap to map normalized heights to RGB values
    colormapSize = numel(unique(highlightedHeights));
    jetColors = jet(colormapSize);
    heightColors = interp1(linspace(0,1,colormapSize), jetColors, normalizedHeights);
    % Apply the calculated colors to the highlighted points
    highlightedPointCloud.Color(highlightedIndices, :) = heightColors * 255; % Colors range [0,255]
    % Update UserData to include the modified point cloud
    f.UserData.pcData = highlightedPointCloud;
    % Update the UIAxes to show the highlighted point cloud
    pcshow(highlightedPointCloud, 'Parent', ax);
end
end

function open_cloud_callback(~, ~, f, ax, option)
    switch option
        case 'Cloud File'
            % Allow multiple file types to be selected
            [file, path] = uigetfile({'*.pcd;*.ply;*.las;*.pts;*.xyz;*.pcd', 'Open Cloud File'});
            if isequal(file, 0)
                return;
            end
            fullPath = fullfile(path, file);
            % Determine the file extension to read the correct point cloud format
            [~, ~, ext] = fileparts(fullPath);
            pc = [];
            switch ext
                case '.ply'
                    pc = pcread(fullPath);
                case '.las'
                    lasReader = lasFileReader(fullPath);
                    lasData = read(lasReader);
                    pc = pointCloud(lasData); % Assuming lasData is appropriately formatted
                case '.pts'
                    % Your logic for reading .pts goes here
                case '.xyz'
                    pcData = dlmread(fullPath);
                    pc = pointCloud(pcData(:, 1:3));
                case '.pcd' % Add this case for PCD files
                    pc = readPCD(fullPath);
                otherwise
                    warning(['File format ', ext, ' is not supported.']);
                    return;
            end
          if ~isempty(pc)
    pcshow(pc, 'Parent', ax);
    f.UserData.pcData = pc;
    f.UserData.originalPcData = pc; % Store the original data

    % Calculate the bounding box dimensions
    pcLocation = pc.Location;
    xMin = min(pcLocation(:, 1));
    xMax = max(pcLocation(:, 1));
    yMin = min(pcLocation(:, 2));
    yMax = max(pcLocation(:, 2));
    zMin = min(pcLocation(:, 3));
    zMax = max(pcLocation(:, 3));
    
if ~isempty(pc)
    pcshow(pc, 'Parent', ax);
    f.UserData.pcData = pc;
    f.UserData.originalPcData = pc; % Store the original data

    % Calculate the bounding box dimensions
    pcLocation = pc.Location;
    xMin = min(pcLocation(:, 1));
    xMax = max(pcLocation(:, 1));
    yMin = min(pcLocation(:, 2));
    yMax = max(pcLocation(:, 2));
    zMin = min(pcLocation(:, 3));
    zMax = max(pcLocation(:, 3));
    
    % Compute length and width in units of the point cloud
    length = xMax - xMin;
    width = yMax - yMin;
    
    % Display raw dimensions in a single line
    disp(['Raw HxW: ', num2str(length), 'x', num2str(width), ' units']);
    
    % Assuming the data might be in millimeters if the dimensions seem too large
    if length > .1 || width > .1
        % Assume data might be in millimeters
        lengthMeters = length / 1000;
        widthMeters = width / 1000;
    else
        % Assume data is already in meters
        lengthMeters = length;
        widthMeters = width;
    end
    
    % Convert from meters to inches
    lengthInches = lengthMeters * 39.3701;
    widthInches = widthMeters * 39.3701;
    
    % Calculate margin of error
    lengthError = lengthInches * 0.02;
    widthError = widthInches * 0.02;
    
    % Display the dimensions and margin of error in a single line
    disp(['WxL: ', num2str(widthInches), 'x', num2str(lengthInches), ' inches ± ', num2str(widthError), 'x', num2str(lengthError), ' inches']);
end
          end
          

        case 'Merge Multiple'
            tic; % Start timing
            
            [files, path] = uigetfile({'*.ply; *.las; *.laz; *.pts; *.xyz; *.asc', 'Merge Point Cloud Files'}, 'MultiSelect', 'on');
            if isequal(files, 0)
                return;
            end

            % Choose which file is for XYZ and which is for Color
            [indx, tf] = listdlg('ListString', files, 'SelectionMode', 'single', 'PromptString', 'Select a file for XYZ coordinates:');
            if tf == 0
                return;
            end
            targetFile = files{indx};
            files(indx) = []; % Remove the selected file from the list
            [indx, tf] = listdlg('ListString', files, 'SelectionMode', 'single', 'PromptString', 'Select a file for Color:');
            if tf == 0
                return;
            end
            colorFile = files{indx};

            % Read the target file for XYZ coordinates
            fullPath = fullfile(path, targetFile);
            [~, ~, ext] = fileparts(fullPath);
            switch ext
                case '.ply'
                    targetPc = pcread(fullPath);
                case '.xyz'
                    pcData = dlmread(fullPath);
                    targetPc = pointCloud(pcData(:, 1:3));
                otherwise
                    warning(['File format ', ext, ' is not supported.']);
                    return;
            end

            % Read the color file for color information
            fullPath = fullfile(path, colorFile);
            [~, ~, ext] = fileparts(fullPath);
            switch ext
                case '.ply'
                    colorPc = pcread(fullPath);
                case '.xyz'
                    colorData = dlmread(fullPath);
                    colorPc = pointCloud(zeros(size(colorData, 1), 3), 'Color', colorData);
                otherwise
                    warning(['File format ', ext, ' is not supported.']);
                    return;
            end

            % Align the color point cloud to the XYZ point cloud
            disp('Starting alignment...');
            [tform, ~, rmse] = pcregrigid(colorPc, targetPc, 'Metric', 'pointToPoint', 'Extrapolate', true);
            disp(['Alignment done. RMSE: ', num2str(rmse)]);

            % Apply transformation to the color point cloud
            colorPcTransformed = pctransform(colorPc, tform);
            disp('Color point cloud transformed.');

            % Extract locations and colors
            targetLocations = double(targetPc.Location);
            colorLocations = double(colorPcTransformed.Location);
            colorValues = double(colorPcTransformed.Color);

            % Visualize aligned point clouds before merging
            figure;
            subplot(1, 2, 1);
            pcshow(targetPc);
            title('Target Point Cloud');
            subplot(1, 2, 2);
            pcshow(colorPcTransformed);
            title('Transformed Color Point Cloud');

            % Normalize the coordinates
            [normalizedTarget, normalizedColor, targetMin, targetRange] = normalize_coordinates(targetLocations, colorLocations);

            % Debugging: Check normalization values before and after
            disp('Target Locations - Before Normalization');
            disp([min(targetLocations); max(targetLocations)]);
            disp('Color Locations - Before Normalization');
            disp([min(colorLocations); max(colorLocations)]);
            disp('Target Locations - After Normalization');
            disp([min(normalizedTarget); max(normalizedTarget)]);
            disp('Color Locations - After Normalization');
            disp([min(normalizedColor); max(normalizedColor)]);

            % Perform IDW interpolation
            disp('Performing IDW interpolation...');
            interpolatedColors = idw_interpolation(normalizedColor, colorValues, normalizedTarget, 2);

            % Ensure all colors are in the valid range [0, 255]
            interpolatedColors = max(0, min(255, interpolatedColors));
            interpolatedColors = uint8(interpolatedColors);

            % Debugging: Verify interpolated colors
            disp('Interpolated Colors - Sample');
            disp(interpolatedColors(1:10, :)); % Display the first 10 interpolated colors

            % Denormalize the coordinates
            denormalizedTarget = denormalize_coordinates(normalizedTarget, targetMin, targetRange);

            % Create a new point cloud with the interpolated colors
            mergedPointCloud = pointCloud(denormalizedTarget, 'Color', interpolatedColors);

            % Display the merged point cloud in the existing window
            cla(ax); % Clear the existing axes
            pcshow(mergedPointCloud, 'Parent', ax);
            axis(ax, 'equal');
            f.UserData.pcData = mergedPointCloud; % Store the merged point cloud
            f.UserData.originalPcData = mergedPointCloud; % Update the original data
            disp(['Merge done. Total points in merged cloud: ', num2str(mergedPointCloud.Count)]);

            % Check if all points have colors assigned
            if all(~isnan(mergedPointCloud.Color), 'all')
                disp('All points in the merged cloud have colors assigned.');
            else
                warning('Some points in the merged cloud do not have colors assigned.');
            end

            % Enable brushing and saving options
            brush on;
            axis(ax, 'equal');
            title(ax, 'Merged Point Cloud');
            f.UserData.saveAllMenu.Enable = 'on';
            f.UserData.saveBrushedMenu.Enable = 'on';
            
            elapsedTime = toc; % End timing
            disp(['Elapsed time: ', num2str(elapsedTime), ' seconds']);

         case 'Mesh File'
            % Prompt user to select a mesh file
            [file, path] = uigetfile({'*.off;*.obj;*.stl;*.dae', 'Open Mesh File'});
            if isequal(file, 0)
                return;
            end
            fullPath = fullfile(path, file);

            % Determine the file extension
            [~, ~, ext] = fileparts(fullPath);

            % Read the mesh file based on its format
            try
                switch ext
                    case '.off'
                        [vertices, faces] = read_off(fullPath);
                    case '.obj'
                        [vertices, faces] = read_obj(fullPath);
                    case '.stl'
                        [vertices, faces] = read_stl(fullPath);
                    case '.dae'
                        [vertices, faces] = read_dae(fullPath);
                    otherwise
                        error('Unsupported mesh file format.');
                end
                
                % Ensure the vertices are in the expected M-by-3 format
                disp(['Vertices size: ', num2str(size(vertices))]); % Debugging
                if size(vertices, 2) ~= 3
                    error('Vertices must be in M-by-3 format.');
                end
                
                % Create a point cloud from vertices
                pc = pointCloud(vertices);

                % Display the point cloud in the existing window
                cla(ax); % Clear the existing axes
                pcshow(pc, 'Parent', ax);
                axis(ax, 'equal');
                f.UserData.pcData = pc; % Store the point cloud
                f.UserData.originalPcData = pc; % Update the original data
                title(ax, 'Mesh Point Cloud');

                % Display a message indicating success
                disp('Mesh file successfully loaded and displayed.');
            catch ME
                warning(['Failed to read mesh file: ', ME.message]);
            end
        otherwise
            warning('Unknown open option');
    end
end

% Helper functions to read mesh file formats
function [vertices, faces] = read_off(filename)
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open the file.');
    end

    header = fgetl(fid);
    if ~strcmp(header, 'OFF')
        error('Not a valid OFF file.');
    end

    counts = fscanf(fid, '%d %d %d', 3);
    numVertices = counts(1);
    numFaces = counts(2);

    vertices = fscanf(fid, '%f %f %f', [3, numVertices])';
    faces = fscanf(fid, '%d %d %d %d', [4, numFaces]);
    faces = faces(2:4, :)' + 1;

    fclose(fid);
end

function [vertices, faces] = read_obj(filename)
    % Read vertices and faces from .obj file
    fid = fopen(filename, 'r');
    vertices = [];
    faces = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, 'v ')
            vertices = [vertices; sscanf(line(3:end), '%f %f %f')'];
        elseif startsWith(line, 'f ')
            faces = [faces; sscanf(line(3:end), '%d %d %d')'];
        end
    end
    fclose(fid);
end

function [vertices, faces] = read_stl(filename)
    % Read vertices and faces from .stl file
    [stl_data, stl_faces, stl_normals] = stlread(filename); % Use MATLAB's built-in stlread function
    vertices = stl_data.Points;
    faces = stl_data.ConnectivityList;
end

function [vertices, faces] = read_dae(filename)
    % Read vertices and faces from .dae file using xmlread and custom parsing
    doc = xmlread(filename);
    vertices = parse_vertices(doc);
    faces = parse_faces(doc);
end

% Custom parsing functions for .dae files
function vertices = parse_vertices(doc)
    % Extract vertices from .dae file
    verticesNode = doc.getElementsByTagName('float_array').item(0);
    verticesText = char(verticesNode.getFirstChild.getData);
    vertices = sscanf(verticesText, '%f');
    vertices = reshape(vertices, 3, [])';
end

function faces = parse_faces(doc)
    % Extract faces from .dae file
    facesNode = doc.getElementsByTagName('p').item(0);
    facesText = char(facesNode.getFirstChild.getData);
    faces = sscanf(facesText, '%d');
    faces = reshape(faces, 3, [])' + 1; % Assuming faces are triangles
end


function [normalizedTarget, normalizedColor, targetMin, targetRange] = normalize_coordinates(target, color)
    % Determine the ranges of the coordinates
    minTarget = min(target, [], 1);
    maxTarget = max(target, [], 1);
    rangeTarget = maxTarget - minTarget;
    
    minColor = min(color, [], 1);
    maxColor = max(color, [], 1);
    rangeColor = maxColor - minColor;
    
    % Normalize the coordinates to the range [0, 1]
    normalizedTarget = (target - minTarget) ./ rangeTarget;
    normalizedColor = (color - minColor) ./ rangeColor;
    
    % Return min and range for inverse normalization later
    targetMin = minTarget;
    targetRange = rangeTarget;
end

function [denormalizedTarget] = denormalize_coordinates(normalized, minTarget, rangeTarget)
    % Denormalize the coordinates back to their original scale
    denormalizedTarget = normalized .* rangeTarget + minTarget;
end

function interpolatedColors = idw_interpolation(colorLocations, colorValues, targetLocations, power)
    % Perform IDW interpolation
    numPoints = size(targetLocations, 1);
    interpolatedColors = zeros(numPoints, 3);
    
    parfor i = 1:numPoints
        distances = sqrt(sum((colorLocations - targetLocations(i, :)).^2, 2));
        weights = 1 ./ (distances .^ power);
        weights = weights / sum(weights);
        interpolatedColors(i, :) = sum(colorValues .* weights, 1);
    end
end
 




    function merge_segments_callback(~, ~, f, ax)
        % Initialize arrays to store merged data
        mergedXYZ = [];
        mergedColor = [];

        while true
            % Allow user to select multiple segment files
            [files, path] = uigetfile({'*.ply; *.las; *.laz; *.pts; *.xyz; *.asc', 'Select Segment Files'}, 'MultiSelect', 'on');
            if isequal(files, 0), break; end

            if ischar(files) % If a single file is selected
                files = {files};
            end

            % Loop through each selected segment file
            for i = 1:length(files)
                segmentFile = fullfile(path, files{i});
                [~, ~, ext] = fileparts(segmentFile);
                switch ext
                    case '.ply'
                        segmentPc = pcread(segmentFile);
                    case '.xyz'
                        segmentData = readmatrix(segmentFile, 'FileType', 'text');
                        segmentPc = pointCloud(segmentData(:, 1:3));
                        % Add other cases if needed
                    otherwise
                        warning(['File format ', ext, ' is not supported.']);
                        continue;
                end
                % Concatenate the XYZ and color data
                mergedXYZ = [mergedXYZ; segmentPc.Location];
                mergedColor = [mergedColor; segmentPc.Color];
            end

            % Prompt user to continue or merge
            choice = questdlg('Do you want to select more segments or merge the current selection?', ...
                'Merge Segments', ...
                'Select More', 'Merge Now', 'Merge Now');
            if strcmp(choice, 'Merge Now')
                break;
            end
        end

        if isempty(mergedXYZ)
            errordlg('No segments were selected for merging.', 'Error');
            return;
        end

        % Create a new point cloud with the merged data
        mergedPointCloud = pointCloud(mergedXYZ, 'Color', mergedColor);
        % Display merged point cloud
        pcshow(mergedPointCloud, 'Parent', ax);
        axis(ax, 'equal');
        f.UserData.pcData = mergedPointCloud; % Ensure pcData is set here
        f.UserData.originalPcData = mergedPointCloud; % Update original data
        disp(['Merge done. Total points in merged cloud: ', num2str(mergedPointCloud.Count)]);
        brush on;
        axis(ax, 'equal');
        title(ax, 'Merged Segment Point Cloud');
        f.UserData.saveAllMenu.Enable = 'on';
        f.UserData.saveBrushedMenu.Enable = 'on';
    end

    function save_ply_callback(~, ~, f, ax, saveMode)
        pcData = f.UserData.pcData;
        % Initialize file and path variables
        file = [];
        path = [];
        if isempty(pcData)
            errordlg('No data to save!', 'Error');
            return;
        end
        % Determine the file extension based on saveMode
        switch saveMode
            case 'all'
                [file, path] = uiputfile('*.ply', 'Save PLY File');
            case 'brushed'
                [file, path] = uiputfile('*.ply', 'Save PLY File');
            otherwise
                errordlg('Unknown save mode!', 'Error');
                if file == 0
                    return; 'No Point'
                end
        end
        fullPath = fullfile(path, file);
        if strcmp(saveMode, 'brushed')
            % Get brushed data
            axChildren = get(ax, 'Children');
            brushedIndices = [];
            for child = 1:length(axChildren)
                if isprop(axChildren(child), 'BrushData')
                    currentBrushedIndices = find(axChildren(child).BrushData);
                    brushedIndices = [brushedIndices; currentBrushedIndices];
                end
            end
            if isempty(brushedIndices)
                errordlg('No points were brushed!', 'Error');
                return;
            end
            % Display the number of brushed points
            disp(['Number of brushed points: ', num2str(length(brushedIndices))]);
            pcData = select(f.UserData.pcData, brushedIndices); % This also keeps RGB and intensity data intact
        end
        % Save the data based on file extension
        [~, ~, option] = fileparts(fullPath);
        switch option
            case '.ply'
                pcwrite(pcData, fullPath, 'Encoding', 'ascii');
            case '.las'
                lasFileWriter(fullPath, pcData.Location, 'Color', uint8(pcData.Color));
            case '.laz'
                % TODO: Implement logic for saving as .laz if different from .las
                warning('LAZ format not yet supported.');
            case '.pts'
                % Save in .pts format
                dataToWrite = [pcData.Location, double(pcData.Color)];
                writematrix(dataToWrite, fullPath, 'Delimiter', ' ');
            case '.xyz'
                % Save in .xyz format
                writematrix(pcData.Location, fullPath, 'Delimiter', ' ');
            case '.asc'
                % Save in .asc format
                fileID = fopen(fullPath, 'w');
                for i = 1:size(pcData.Location, 1)
                    fprintf(fileID, '%f %f %f\n', pcData.Location(i, 1), pcData.Location(i, 2), pcData.Location(i, 3));
                end
                fclose(fileID);
            otherwise
                errordlg('Unsupported file type!', 'Error');
        end
    end
