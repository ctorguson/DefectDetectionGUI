function bGui()
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
newFormats = {'3D File', 'Merge Multiple'};
for i = 1:length(newFormats)
 uimenu(openMenu, 'Label', newFormats{i}, 'Callback', @(src, event) open_cloud_callback(src, event, f, ax, newFormats{i}));
end
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
 % Fit a parabolic surface of the form z = ax^2 + by^2 + c
 % to the point cloud data
 X = [pcData(:,1).^2, pcData(:,2).^2, ones(size(pcData,1),1)];
 abc = X \ pcData(:,3); % Least squares solution
 % Compute the z-values of the fitted parabolic surface
 fittedZ = abc(1)*pcData(:,1).^2 + abc(2)*pcData(:,2).^2 + abc(3);
 % Subtract the fitted parabolic surface from the original z-values
 % to flatten the primary curvature
 flattenedZ = pcData(:,3) - fittedZ;
 % Create the new point cloud data with the flattened z-values
 flattenedPoints = [pcData(:,1:2), flattenedZ];
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
grid(ax, 'on');
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
% Logic for "Defect" option
% function defect_option_callback(~, ~, f, ax)
% neuralNetModel = f.UserData.neuralNetModel; % Retrieve the model from UserData
% if isempty(f.UserData.pcData)
% errordlg('No data to process!', 'Error');
% return;
% end
% Get the point cloud data
% pcData = f.UserData.pcData;
% Here, make a call to neural network model to get predictions
% Preprocess data as required and then use the model to predict
% For example:
% predictions = classify(neuralNetModel, pcData.Location);
% Assuming predictions is a binary matrix where 1 indicates defects
% defectIndices = find(predictions == 1);
% defectPoints = select(pcData, defectIndices);
% Highlight defects based on Z-depth
% z = defectPoints.Location(:, 3);
% colormap(ax, 'jet'); % Using jet colormap, adjust as needed
% pcshow(defectPoints, 'Parent', ax, 'CData', z);
% end
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
 case '3D File'
 % Allowing multiple file types to be selected
 [file, path] = uigetfile({'*.pcd;*.ply;*.las;*.pts;*.xyz;*.pcd', 'Point Cloud Files'}, 'Open Cloud File');
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
 %case '.obj'
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
 end
 case 'Merge Multiple'
 [files, path] = uigetfile({'*.ply; *.las; *.laz; *.pts; *.xyz; *.asc', 'Point Cloud Files'}, 'Merge Point Cloud Files', 'MultiSelect', 'on');
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
 % Add other cases if needed
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
 % If the color file is also .xyz format, interpret it as RGB values
 colorData = dlmread(fullPath);
 colorPc = pointCloud(zeros(size(colorData, 1), 3), 'Color', colorData);
 % Add other cases if needed
 otherwise
 warning(['File format ', ext, ' is not supported.']);
 return;
 end
 % If the number of points do not match, interpolate color from colorPc to targetPc using nearest neighbors
 if targetPc.Count ~= colorPc.Count
 % Construct a KD-tree for the color point cloud
 Mdl = KDTreeSearcher(colorPc.Location);
 % For every point in targetPc, find the closest point in colorPc
 idx = knnsearch(Mdl, targetPc.Location);
 % Assign colors from the nearest neighbors
 mergedColors = colorPc.Color(idx, :);
 else
 mergedColors = colorPc.Color;
 end
 % Create a new point cloud with merged data
 mergedPointCloud = pointCloud(targetPc.Location, 'Color', mergedColors);
 % Display merged point cloud
 pcshow(mergedPointCloud, 'Parent', ax);
 axis(ax, 'equal');
 f.UserData.mergedPc = mergedPointCloud;
 switch ext
 case '.ply'
 % Read the target file for XYZ coordinates
 tempTargetPc = pcread(fullPath);
 targetPcStruct.Location = tempTargetPc.Location;
 targetPcStruct.Color = tempTargetPc.Color;
 % Add other cases if needed
 otherwise
 warning(['File format ', ext, ' is not supported.']);
 return;
 end
 % Read the color file for color information
 fullPath = fullfile(path, colorFile);
 [~, ~, ext] = fileparts(fullPath);
 switch ext
 case '.ply'
 % Read the color file for color information
 tempColorPc = pcread(fullPath);
 colorPcStruct.Location = tempColorPc.Location;
 colorPcStruct.Color = tempColorPc.Color;
 % Add other cases if needed
 otherwise
 warning(['File format ', ext, ' is not supported.']);
 return;
 end
 if isempty(targetPc) || isempty(colorPc)
 return;
 end
 % Assume both point clouds are pre-aligned. If not, you may have to align them first.
 % Convert structs to pointCloud objects
 targetPc = pointCloud(targetPcStruct.Location, 'Color', targetPcStruct.Color);
 colorPc = pointCloud(colorPcStruct.Location, 'Color', colorPcStruct.Color);
 % Merge the two point clouds
 % Merge XYZ and color data
 mergedXYZ = targetPc.Location;
 mergedColor = colorPc.Color;
 % You can also add additional processing here if necessary,
 % like downsampling, aligning, or filtering.
 % Use ICP to fine-tune the alignment
 [tform,~,~] = pcregrigid(colorPc, targetPc, 'Metric','pointToPoint', 'Extrapolate', true);
 % Transform the color point cloud to align with the target point cloud
 colorPcTransformed = pctransform(colorPc, tform);
 % Merge XYZ and color data
 mergedXYZ = [targetPc.Location; colorPcTransformed.Location]; % concatenate the XYZ coordinates
 mergedColor = [targetPc.Color; colorPcTransformed.Color]; % concatenate the color data
 % ----------------------------- NEW LINES END -------------------------------
 % Check if dimensions match
 if size(mergedXYZ, 1) ~= size(mergedColor, 1)
 warning('Number of points in XYZ data does not match number of color points.');
 return;
 end
 mergedPointCloud = pointCloud(mergedXYZ, 'Color', mergedColor);
 % Show the merged point cloud and store in UserData
 if ~isempty(mergedPointCloud)
 pcshow(mergedPointCloud, 'Parent', ax);
 f.UserData.pcData = mergedPointCloud;
 f.UserData.originalPcData = mergedPointCloud;
 end
 brush on;
 axis(ax, 'equal');
 title(ax, 'Merged Point Cloud');
 f.UserData.saveAllMenu.Enable = 'on';
 f.UserData.saveBrushedMenu.Enable = 'on';
 otherwise
 warning('Unknown open option');
end
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
