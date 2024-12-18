%=======================================================================
%  New versioning section.  Please update here when making changes.
% 
%  Revision date and number:  2024-10-07      Version 1
%  
%  Version 1 date:      2024-10-07
%  Version 1 changes:   Cleaned up temporary fix for mis-scaled ATOS data.
%                       Added figures for interim steps.
%                       Added drawnow() command after each figure so we 
%                        can see the figures while next step continues.
%
%  Version 1A date:     2024-10-14
%  Version 1A changes:  General clean-up and edits
%  
%  
%  Version 2 date:      2024-10-30
%  Version 2 changes:   Changed color mapping algo to use the mink()
%                       function rather than sort() since we only care
%                       about the closest few points.  This resulted in a
%                       speed-up of about 5x.  However, it's still only
%                       good for relatively small files ( <500,000 pts).
%  
%  
%  Version 3 date:      2024-10-30
%  Version 3 changes:   Removed extra figure, displayed merged cloud in main GUI window.
%                       Added Parpool parallel pool for intepolation speed.
%                       
%  
%  Version 4 date:      yyyy-mm-dd
%  Version 4 changes:   List updates here
%=======================================================================
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
% Ensure that point cloud data is available in the GUI's UserData
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    % Access point cloud data from UserData
    ptCloud = f.UserData.pcData;

    % Downsample to ~5000-5500 points
    gridStep = 0.005;
    pcDownsize = pcdownsample(ptCloud, 'gridAverage', gridStep);

    % Grab downsized x, y, z coordinates into arrays
    X_down = cast(pcDownsize.Location(:,1), "double");
    Y_down = cast(pcDownsize.Location(:,2), "double");
    Z_down = cast(pcDownsize.Location(:,3), "double");

    % Fit a 3rd order polynomial to the downsized point cloud data
    polyFit = fit([X_down, Y_down], Z_down, 'poly33');
    p00 = polyFit.p00;
    p10 = polyFit.p10;
    p01 = polyFit.p01;
    p20 = polyFit.p20;
    p11 = polyFit.p11;
    p02 = polyFit.p02;
    p30 = polyFit.p30;
    p21 = polyFit.p21;
    p12 = polyFit.p12;
    p03 = polyFit.p03;

    % Calculate the modifier array based on the polynomial fit
    modifier_array = p00 + p10 .* X_down + p01 .* Y_down + p20 .* (X_down.^2) ...
        + p11 .* X_down .* Y_down + p02 .* (Y_down.^2) + p30 .* (X_down.^3) ...
        + p21 .* (X_down.^2) .* Y_down + p12 .* X_down .* (Y_down.^2) + p03 .* (Y_down.^3);

    % Update the Z values
    Z_new = Z_down - modifier_array;

    % Create a new point cloud with flattened Z values
    pcMatrix = [X_down, Y_down, Z_new];
    pcFiltered = pointCloud(pcMatrix); % Generate filtered point cloud

    % Remove outliers using 3-sigma rule
    pcStats = datastats(Z_new);
    stDev = pcStats.std;
    meanVal = pcStats.mean;
    lower_bound = meanVal - (3 * stDev);
    upper_bound = meanVal + (3 * stDev);

    X_intermediate = X_down;
    Y_intermediate = Y_down;
    Z_intermediate = Z_new;

    X_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
    Y_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
    Z_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];

    % Create a point cloud with no outliers
    pcMatrix2 = [X_intermediate, Y_intermediate, Z_intermediate];
    pcNoOutliers = pointCloud(pcMatrix2);

    % Display the point cloud without outliers
    pcshow(pcNoOutliers, 'Parent', ax);
    title(ax, 'Filtered Point Cloud (No Outliers)');

    % Perform a second polynomial fit without outliers
    polyFit2 = fit([X_intermediate, Y_intermediate], Z_intermediate, 'poly33');
    p00_final = polyFit2.p00;
    p10_final = polyFit2.p10;
    p01_final = polyFit2.p01;
    p20_final = polyFit2.p20;
    p11_final = polyFit2.p11;
    p02_final = polyFit2.p02;
    p30_final = polyFit2.p30;
    p21_final = polyFit2.p21;
    p12_final = polyFit2.p12;
    p03_final = polyFit2.p03;

    % Grab original x, y, z, and color values
    X_original = cast(ptCloud.Location(:,1), "double");
    Y_original = cast(ptCloud.Location(:,2), "double");
    Z_original = cast(ptCloud.Location(:,3), "double");
    R_original = cast(ptCloud.Color(:,1), "uint8");
    G_original = cast(ptCloud.Color(:,2), "uint8");
    B_original = cast(ptCloud.Color(:,3), "uint8");

    % Apply the second polynomial fit to the original data
    num2 = length(X_original);

    p00_array2 = p00_final * ones(num2, 1);
    p10_array2 = p10_final .* X_original;
    p01_array2 = p01_final .* Y_original;
    p20_array2 = p20_final .* (X_original.^2);
    p11_array2 = p11_final .* X_original .* Y_original;
    p02_array2 = p02_final .* (Y_original.^2);
    p30_array2 = p30_final .* (X_original.^3);
    p21_array2 = p21_final .* (X_original.^2) .* Y_original;
    p12_array2 = p12_final .* X_original .* (Y_original.^2);
    p03_array2 = p03_final .* (Y_original.^3);

    modifier_array2 = p00_array2 + p10_array2 + p01_array2 + p20_array2 + p11_array2 ...
        + p02_array2 + p30_array2 + p21_array2 + p12_array2 + p03_array2;

    % Final Z values after second polynomial fit
    Z_final = Z_original - modifier_array2;

    % Generate the final flattened point cloud
    xyz = [X_original, Y_original, Z_final];
    rgb = [R_original, G_original, B_original];
    pcFinal = pointCloud(xyz, 'Color', rgb);

    % Display the final point cloud
    pcshow(pcFinal, 'Parent', ax);
    title(ax, 'Final Flattened Point Cloud');

    % Optionally save the final point cloud (if needed, otherwise remove)
    % pcwrite(pcFinal, 'final_flattened_data.ply');

    % Store the final processed point cloud in UserData for future use
    f.UserData.pcFinal = pcFinal;

else
    % Error handling if point cloud is not available
    errordlg('No point cloud data available to flatten!', 'Error');
end
end

function mesh_option_callback(~, ~, f, ax)
%   Check if point cloud data exists
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

                % Compute length, width, and height in inches
                length = xMax - xMin;
                width = yMax - yMin;
                height = zMax - zMin;

                % Convert dimensions directly to inches (assuming input is already in inches)
                lengthInches = length;
                widthInches = width;
                heightInches = height;

                % Display the dimensions in separate lines with proper unit names
                disp(['Length: ', num2str(lengthInches), ' inches']);
                disp(['Width: ', num2str(widthInches), ' inches']);
                disp(['Height: ', num2str(heightInches), ' inches']);
            end

case 'Merge Multiple'

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
    [tform, ~, rmse] = pcregrigid(colorPc, targetPc, 'Metric', 'pointToPoint',...
             'Extrapolate', true,'Tolerance',[0.0001,0.009], ...
             'MaxIterations',100);
    disp(['Alignment done. RMSE: ', num2str(rmse)]);

    % Create a figure to show the cloud after alignment
    figure('Name', 'Aligned Color Point Cloud');
    alignedPc = pctransform(colorPc, tform);
    pcshow(alignedPc);
    title('Aligned Color Point Cloud');

    % Apply transformation to the color point cloud
    colorPcTransformed = pctransform(colorPc, tform);
    disp('Color point cloud transformed.');

    % Create a figure to show the cloud after color point cloud transformation
    figure('Name', 'Color Point Cloud after Transformation');
    pcshow(colorPcTransformed);
    title('Color Point Cloud after Transformation');

    % Extract locations and colors
    targetLocations = double(targetPc.Location); % Define targetLocations as a variable
    colorLocations = double(colorPcTransformed.Location);
    colorValues = double(colorPcTransformed.Color);

    % Perform batch parallel processing for color interpolation
    disp('Performing batch parallel processing for color interpolation...');
    tic; % Start timing

    numTargets = size(targetLocations, 1);
    batchSize = 1000; % Define an appropriate batch size based on your data size
    numBatches = ceil(numTargets / batchSize);
    interpolatedColors = zeros(numTargets, 3, 'uint8'); % Pre-allocate for RGB

    % Create a cell array to temporarily store results from each batch
    batchResults = cell(numBatches, 1);

    % Parallel batch processing loop
    parfor b = 1:numBatches
        % Define the indices for the current batch
        startIdx = (b - 1) * batchSize + 1;
        endIdx = min(b * batchSize, numTargets);
        batchIndices = startIdx:endIdx;
        
        % Process each batch
        batchTargetLocations = targetLocations(batchIndices, :);
        batchInterpolatedColors = zeros(numel(batchIndices), 3, 'uint8');
        
        % Loop over each point in the batch (serial within each batch)
        for i = 1:numel(batchIndices)
            batchInterpolatedColors(i, :) = epm_interpolation(colorLocations, colorValues, batchTargetLocations(i, :));
        end
        
        % Store the batch results in the cell array
        batchResults{b} = batchInterpolatedColors;
    end

    % After the parallel loop, combine results from all batches
    for b = 1:numBatches
        startIdx = (b - 1) * batchSize + 1;
        endIdx = min(b * batchSize, numTargets);
        interpolatedColors(startIdx:endIdx, :) = batchResults{b};
    end

    % Display total elapsed time only once
    disp(['Total elapsed time for interpolation: ', num2str(toc), ' seconds']);

    % Ensure all colors are in the valid range [0, 255]
    interpolatedColors = max(0, min(255, interpolatedColors));

    % Create a new point cloud with the interpolated colors
    mergedPointCloud = pointCloud(targetLocations, 'Color', interpolatedColors);

    % Display the merged point cloud in the existing main window
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

function interpolatedColors = epm_interpolation(colorLocations, colorValues, targetLocations)
    % Perform interpolation
    tic
    numPoints = size(targetLocations, 1);
    numPoints10 = round(numPoints/10);
    interpolatedColors = zeros(numPoints, 3);
    num_avg = 8;
    for i = 1:numPoints
        if mod(i,numPoints10)==1
            fprintf('%d out of %d colors set\n',i,numPoints)
            toc
        end
        distances = sqrt(sum((colorLocations - targetLocations(i,:)).^2, 2));
        % [~,idx] = sort(distances);
        % nearlist = idx(1:num_avg);
        % interpolatedColors(i, :) = sum(colorValues(nearlist,:)) / num_avg;


        [s,idx] = mink(distances,num_avg);
        interpolatedColors(i, :) = sum(colorValues(idx,:)) / num_avg;
    end
    toc
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
