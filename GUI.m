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
%  Version 2 date:     2024-10-14
%  Version 2 changes:  General clean-up and edits
%
%
%  Version 3 date:      2024-10-30
%  Version 3 changes:   Removed extra figure, displayed merged cloud in main GUI window.
%                       Added Parpool parallel pool for intepolation speed.
%
%  Version 4 date:      2025-01-14
%  Version 4 changes:   Added defect detection function.
%
%
%  Version 5 date:      2025-01-15
%  Version 5 changes:   Added Pre-Align function.
%
%  Version 5 date:      2025-01-22
%  Version 5 changes:   Added user defined number of grid spaces in the
%                       "Grid" visualization option.
%
%  Version # date:      yyyy-mm-dd
%  Version # changes:   List updates here
%=======================================================================
function GUI()
% Create figure
f = figure('Name', 'Point Cloud Window', 'NumberTitle', 'off');
ax = axes(f);
set(ax, 'ButtonDownFcn', @(src, event) axes_ButtonDownFcn(src, event, f));

    function axes_ButtonDownFcn(~, event, f)
        % Ensure 'pcData' field exists in UserData before proceeding
        if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
            % Get the current point clicked
            currentPoint = event.IntersectionPoint(1:3);
            % Find the closest point in your point cloud
            Mdl = KDTreeSearcher(f.UserData.pcData.Location);
            closestIndex = knnsearch(Mdl, currentPoint, 'K', 1);
            % Store the index of the closest point in UserData
            f.UserData.selectedPointIndex = closestIndex;
        else
            % Suppress error and provide non-intrusive feedback
            disp('No point cloud data available. Please load a file first.');
        end
    end

% Dropdown Menu for Opening Clouds
openMenu = uimenu(f, 'Label', 'Open');
newFormats = {'Cloud File', 'Merge Multiple', 'Mesh File', 'Pre-Align'};
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
end

function flatten_parabolic_callback(~, ~, f, ax)
% Ensure that point cloud data is available in the GUI's UserData
if isfield(f.UserData, 'pcData') && ~isempty(f.UserData.pcData)
    % Access point cloud data from UserData
    ptCloud = f.UserData.pcData;

    % Downsample to ~5000-5500 points
    gridStep = 0.005;
    pcDownsize = pcdownsample(ptCloud, 'gridAverage', gridStep);
    pcshow(pcDownsize)

    % Grab downsized x, y, z coordinates into arrays
    X_down = cast(pcDownsize.Location(:,1), "double");
    Y_down = cast(pcDownsize.Location(:,2), "double");
    Z_down = cast(pcDownsize.Location(:,3), "double");

    % Fit a 3rd order polynomial to the downsized point cloud data
    % f(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 +...
    % p21*x^2*y + p12*x*y^2 + p03*y^3
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

    %grab length of downsized PC
    num = length(X_down);

    p00_array = p00 * ones(num, 1);
    p10_array = p10 .* X_down;
    p01_array = p01 .* Y_down;
    p20_array = p20 .* (X_down.^2);
    p11_array = p11 .* X_down .* Y_down;
    p02_array = p02 .*(Y_down.^2);
    p30_array = p30 .* (X_down.^3);
    p21_array = p21 .* (X_down.^2) .* Y_down;
    p12_array = p12 .* X_down .* (Y_down.^2);
    p03_array = p03 .* (Y_down.^3);

    modifier_array = p00_array + p10_array + p01_array + p20_array + p11_array + p02_array + p30_array + p21_array + p12_array + p03_array;

    % Update the Z values
    Z_new = Z_down - modifier_array;

    % Remove outliers using 3-sigma rule
    pcStats = datastats(Z_new);
    stDev = pcStats.std;
    meanVal = pcStats.mean;
    lower_bound = meanVal - (3 * stDev);
    upper_bound = meanVal + (3 * stDev);

    X_intermediate = X_down;
    Y_intermediate = Y_down;
    Z_intermediate = Z_down;

    X_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
    Y_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
    Z_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];

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

    modifier_array2 = p00_array2 + p10_array2 + p01_array2 + p20_array2 + p11_array2 + p02_array2 + p30_array2 + p21_array2 + p12_array2 + p03_array2;

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

function [flatPC] = flatten_function(ptCloud)

% Downsample to ~5000-5500 points
gridStep = 0.005;
pcDownsize = pcdownsample(ptCloud, 'gridAverage', gridStep);
pcshow(pcDownsize)

% Grab downsized x, y, z coordinates into arrays
X_down = cast(pcDownsize.Location(:,1), "double");
Y_down = cast(pcDownsize.Location(:,2), "double");
Z_down = cast(pcDownsize.Location(:,3), "double");

% Fit a 3rd order polynomial to the downsized point cloud data
% f(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2 + p30*x^3 +...
% p21*x^2*y + p12*x*y^2 + p03*y^3
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

%grab length of downsized PC
num = length(X_down);

p00_array = p00 * ones(num, 1);
p10_array = p10 .* X_down;
p01_array = p01 .* Y_down;
p20_array = p20 .* (X_down.^2);
p11_array = p11 .* X_down .* Y_down;
p02_array = p02 .*(Y_down.^2);
p30_array = p30 .* (X_down.^3);
p21_array = p21 .* (X_down.^2) .* Y_down;
p12_array = p12 .* X_down .* (Y_down.^2);
p03_array = p03 .* (Y_down.^3);

modifier_array = p00_array + p10_array + p01_array + p20_array + p11_array + p02_array + p30_array + p21_array + p12_array + p03_array;

% Update the Z values
Z_new = Z_down - modifier_array;

% Remove outliers using 3-sigma rule
pcStats = datastats(Z_new);
stDev = pcStats.std;
meanVal = pcStats.mean;
lower_bound = meanVal - (3 * stDev);
upper_bound = meanVal + (3 * stDev);

X_intermediate = X_down;
Y_intermediate = Y_down;
Z_intermediate = Z_down;

X_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
Y_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];
Z_intermediate(Z_new < lower_bound | Z_new > upper_bound) = [];

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
flatPC = pointCloud(xyz, 'Color', rgb);
end

function defect_option_callback(~, ~, f, ax)
% Check if the point cloud exists
if isfield(f.UserData, 'originalPcData') && ~isempty(f.UserData.originalPcData)
    % Retrieve the point cloud
    ptCloud = f.UserData.originalPcData;

    % Perform defect detection
    defect = grid_defect_detection(ptCloud);

    % Create a copy for visualization
    defect_pc = copy(ptCloud);

    % Apply colors to defect regions based on defect type
    for p = 1:length(defect)
        defect_tile = defect(p);
        roi = defect_tile.bounds;
        defect_type = defect_tile.type;

        % Skip processing if no defect
        if strcmp(defect_type, 'no defect') || strcmp(defect_type, 'empty')
            continue;
        end

        indices = findPointsInROI(defect_pc, roi);

        % Assign colors based on defect type
        if strcmp(defect_type, 'correct fastener') || strcmp(defect_type, 'tooling ball')
            defect_pc.Color(indices, 2) = 255; % Make green
        elseif strcmp(defect_type, 'damaged fastener') || strcmp(defect_type, 'sharpie')
            defect_pc.Color(indices, 1) = 255; % Make yellow
            defect_pc.Color(indices, 2) = 255;
        elseif strcmp(defect_type, 'scratch') || strcmp(defect_type, 'dig')
            defect_pc.Color(indices, 1) = 255; % Make red
        end
    end

    % Display the updated point cloud
    pcshow(defect_pc, 'Parent', ax);
    f.UserData.pcData = defect_pc; % Save the updated point cloud

else
    disp('No point cloud data available.');
end
end

function [defect] = grid_defect_detection(ptCloud)
% Main defect detection function

pcData = ptCloud;

% Flatten the point cloud
pcFlattened = flatten_function(pcData);

% Calculate the bounding box of the point cloud
xMin = min(pcFlattened.Location(:, 1));
xMax = max(pcFlattened.Location(:, 1));
yMin = min(pcFlattened.Location(:, 2));
yMax = max(pcFlattened.Location(:, 2));

% Define grid resolution
numSquaresX = ceil(xMax - xMin); % One-inch tiles
numSquaresY = ceil(yMax - yMin);

Zmin = -1000;
Zmax = 1000;

defect = []; % Structure for storing defects
count = 1;

% Loop over each grid tile
for i = 1:numSquaresY
    for j = 1:numSquaresX
        Xbound1 = xMin + (j - 1);
        Xbound2 = Xbound1 + 1;
        Ybound1 = yMin + (i - 1);
        Ybound2 = Ybound1 + 1;
        roi = [Xbound1 Xbound2 Ybound1 Ybound2 Zmin Zmax];

        indices = findPointsInROI(pcFlattened, roi);
        InspectionTile = select(pcFlattened, indices);
        numPoints = InspectionTile.Count;
        if numPoints < 10000 % minimum number of points to run a defect analysis (may be modified)
            defect(count).bounds = roi;
            defect(count).type = 'empty';
        else
            result = surf_calc(InspectionTile);
            defect(count).bounds = roi;
            defect(count).type = result;
        end
        count = count + 1;
    end
end
end

function [result] = surf_calc(tile)
% Perform surface analysis for defect classification

% PARAMETERS USED:
% Sp_Height
% Sp_Blue
% Sv_Height
% Sz_Height
% Sxp_Height
% Sxp_Red
% Sdq_Red

% Generate matrix of equally spaced points for surf parameter calcs
% (dont need green values for calculations)
our_pcx = cast(tile.Location(:,1), "double");
our_pcy = cast(tile.Location(:,2), "double");
our_pcz = cast(tile.Location(:,3), "double");
our_pcR = cast(tile.Color(:,1), "double");
our_pcB = cast(tile.Color(:,3), "double");

% Generate equally spaced matrix of x and y values
[xmin, xmax]=bounds(our_pcx);
[ymin, ymax]=bounds(our_pcy);
max_wdth = max (xmax-xmin, ymax-ymin);
pix_size = max_wdth/4000;
[xq, yq] = meshgrid(xmin : pix_size : xmax , ymin : pix_size : ymax);
[ySize, xSize] = size(xq);
xSize = int32(xSize);
ySize = int32(ySize);

% Generate interpolated R height data
newZData = griddata(our_pcx, our_pcy, our_pcz , xq , yq);
newRData = griddata(our_pcx, our_pcy, our_pcR , xq , yq);
newBData = griddata(our_pcx, our_pcy, our_pcB , xq , yq);

% Calculate Peak Parameters
Sp_Height = max(newZData, [], 'all') * 25400; % Multiply to get parameter in microns
Sp_Blue = max(newBData, [], 'all');
Sv_Height = min(newZData, [], 'all') * 25400; % Multiply to get parameter in microns
Sz_Height = Sp_Height - Sv_Height;

% Calculate Sxp Parameters
Heights_List = newZData(:);
Heights_List = Heights_List .* 25400; % Multiply to get heights in microns
Heights_Sorted = sort(Heights_List, 'descend');
Reds_List = newRData(:);
Reds_Sorted = sort(Reds_List, 'descend');
num = numel(Heights_Sorted);
fifty = floor(num/2);
two_point_five = floor(num/40);

Height_fifty = Heights_Sorted(fifty);
Height_two_point_five = Heights_Sorted(two_point_five);
Red_fifty = Reds_Sorted(fifty);
Red_fifty = cast(Red_fifty, 'double'); % Convert uint8 to double for subtraction
Red_two_point_five = Reds_Sorted(two_point_five);
Red_two_point_five = cast(Red_two_point_five, 'double'); % Convert uint8 to double for subtraction

Sxp_Height = Height_fifty - Height_two_point_five;
Sxp_Red = Red_fifty - Red_two_point_five;

% Calculate Sdq for Red
Sdq_Red = sdq_calc(newRData, xSize, ySize, pix_size);

% Follow decision tree to determine which defect type
result = decision_tree(Sp_Height, Sp_Blue, Sv_Height, Sz_Height, Sxp_Height, Sxp_Red, Sdq_Red);
end

function defect_type = decision_tree(Sp_Height, Sp_Blue, Sv_Height, Sz_Height, Sxp_Height, Sxp_Red, Sdq_Red)
% Decision tree for defect classification
if Sz_Height <= 138.43
    if Sp_Blue <= 162.5
        if Sdq_Red <= 87.85
            if Sv_Height <= 57.74
                defect_type = 'scratch';
            else
                defect_type = 'correct fastener';
            end
        else
            if Sxp_Red <= -13.12
                defect_type = 'scratch';
            else
                defect_type = 'correct fastener';
            end
        end
    else
        if Sxp_Height <= -18.58
            if Sp_Blue <= 166.92
                defect_type = 'scratch';
            else
                defect_type = 'tooling ball';
            end
        else
            if Sv_Height <= -53.14
                defect_type = 'sharpie';
            else
                defect_type = 'scratch';
            end
        end
    end
else
    if Sz_Height <= 846.22
        if Sz_Height <= 368.23
            defect_type = 'tooling ball';
        else
            defect_type = 'no defect';
        end
    else
        if Sp_Height <= 737.25
            defect_type = 'damaged fastener';
        else
            if Sdq_Red <= 75.23
                defect_type = 'scratch';
            else
                defect_type = 'dig';
            end
        end
    end
end
end

function [Sdq_Red] = sdq_calc(newRData, xSize, ySize, pix_size)
% Calculate root-mean-square gradient (Sdq) for the red channel of the point cloud
sum = 0;
numPoints = 0;

% Calculate X and Y partial derivatives for each point in the tile
for m = 1:xSize
    for n = 1:ySize

        % calculate whether neighboring data points are NaN
        if m > 1
            mMinusOne = isnan(newRData(n, m-1));
        else
            mMinusOne = 1;
        end

        if m < xSize
            mPlusOne = isnan(newRData(n, m+1));
        else
            mPlusOne = 1;
        end

        if n > 1
            nMinusOne = isnan(newRData(n-1, m));
        else
            nMinusOne = 1;
        end

        if n < ySize
            nPlusOne = isnan(newRData(n+1,m));
        else
            nPlusOne = 1;
        end

        if isnan(newRData(n,m)) == true % if data point is NaN, skip
            break % skip data point

        else
            pixel = 1;

            % Calculate X Partial
            if mMinusOne == true && mPlusOne == false % left x value is NaN, right is a number
                xPartial = (newRData(n, m+1) - newRData(n,m))^2; % forward differencing
                pixel = pixel / 2;
            elseif mMinusOne == false && mPlusOne == true % left x value is a real number, right is NaN
                xPartial = (newRData(n,m) - newRData(n, m-1))^2; % backward differencing
                pixel = pixel / 2;
            elseif mMinusOne == false && mPlusOne == false % both neighbors are real numbers
                xPartial = ((newRData(n, m+1) - newRData(n, m-1)) / 2)^2; % centered differencing

            else
                xPartial = NaN;
            end

            % Calculate Y Partial
            if nMinusOne == true && nPlusOne == false % left y value is NaN, right is a number
                yPartial = (newRData(n+1, m) - newRData(n, m))^2; % forward differencing
                pixel = pixel / 2;
            elseif nMinusOne == false && nPlusOne == true % left y value is a real number, right is NaN
                yPartial = (newRData(n, m) - newRData(n-1, m))^2; % backward differencing
                pixel = pixel / 2;
            elseif nMinusOne == false && nPlusOne == false % both neighbors are real numbers
                yPartial = ((newRData(n+1, m) - newRData(n-1, m)) / 2)^2; % centered differencing

            else
                yPartial = NaN;
            end

            % If the x and y partials are real numbers, add them to the
            % sum of the partials.
            % NumPoints is used to calculate the area used for Sdq
            % calculation
            if isnan(xPartial) == false && isnan(yPartial) == false
                sum = sum + xPartial + yPartial;
                numPoints = numPoints + pixel;
            end
        end
    end
end

% estimate total area: number of pixels used to calculate Sdq * the area of each pixel (square pixels)
area = numPoints * (pix_size ^ 2) * 645.16; % Calculate area (in^2) and convert to mm^2
Sdq_Red = sqrt(sum / area);
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

function vertexColors = interpolate_vertex_colors(colorData, ~, pts)
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

% Create input dialog for grid settings
prompt = {'Enter number of grid spaces in X direction:', ...
    'Enter number of grid spaces in Y direction:', ...
    'Enter number of grid spaces in Z direction:'};
dlgtitle = 'Grid Options';
dims = [1 50];
definput = {'4', '4', '3'}; % Default values
answer = inputdlg(prompt, dlgtitle, dims, definput);

if isempty(answer)
    return; % Exit if user cancels input
end

% Convert input strings to numbers
numSquaresX = str2double(answer{1});
numSquaresY = str2double(answer{2});
numSquaresZ = str2double(answer{3});

if isnan(numSquaresX) || isnan(numSquaresY) || isnan(numSquaresZ) || ...
        numSquaresX <= 0 || numSquaresY <= 0 || numSquaresZ <= 0
    errordlg('Invalid input! Please enter positive integers.', 'Error');
    return;
end

% Calculate the spacing between grid lines
xSpacing = (xMax - xMin) / numSquaresX;
ySpacing = (yMax - yMin) / numSquaresY;
zSpacing = (zMax - zMin) / numSquaresZ;

% Calculate grid line positions
xLines = xMin:xSpacing:xMax;
yLines = yMin:ySpacing:yMax;
zLines = zMin:zSpacing:zMax;

% Remove any existing grid lines before drawing new ones
delete(findobj(ax, 'Type', 'line', 'Tag', 'GridLine'));

% Draw vertical grid lines along the X direction
for x = xLines
    for z = zLines
        line(ax, [x x], [yMin yMax], [z z], 'Color', 'k', 'LineWidth', 1, 'Tag', 'GridLine');
    end
end

% Draw horizontal grid lines along the Y direction
for y = yLines
    for z = zLines
        line(ax, [xMin xMax], [y y], [z z], 'Color', 'k', 'LineWidth', 1, 'Tag', 'GridLine');
    end
end

% Draw depth grid lines along the Z direction
for z = zLines
    for x = xLines
        line(ax, [x x], [yMin yMax], [z z], 'Color', 'k', 'LineWidth', 1, 'Tag', 'GridLine');
    end
    for y = yLines
        line(ax, [xMin xMax], [y y], [z z], 'Color', 'k', 'LineWidth', 1, 'Tag', 'GridLine');
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
    rotAxis = cross(normal, [0 0 1]); % Rotation axis
    rotAngle = acos(dot(normal, [0 0 1])); % Rotation angle

    % Normalize the rotation axis
    if norm(rotAxis) > 1e-6  % Avoid division by zero
        rotAxis = rotAxis / norm(rotAxis);
        % Rodrigues' rotation formula
        K = [0 -rotAxis(3) rotAxis(2);
            rotAxis(3) 0 -rotAxis(1);
            -rotAxis(2) rotAxis(1) 0];

        rotMatrix = eye(3) + sin(rotAngle) * K + (1 - cos(rotAngle)) * (K * K);
    else
        rotMatrix = eye(3); % No rotation needed if axis is already aligned
    end

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

        disp('Starting alignment...');
        [tform, ~, rmse] = pcregrigid(colorPc, targetPc, 'Metric', 'pointToPoint',...
            'Extrapolate', true,'Tolerance',[0.0001,0.009], ...
            'MaxIterations',100);
        disp(['Alignment done. RMSE: ', num2str(rmse)]);

        % Apply transformation to the color point cloud
        colorPcTransformed = pctransform(colorPc, tform);
        disp('Color point cloud transformed.');

        % Extract locations and colors
        targetLocations = double(targetPc.Location);
        colorLocations = double(colorPcTransformed.Location);
        colorValues = double(colorPcTransformed.Color);

        % Visualize point clouds before merging
        figure;
        subplot(1, 3, 1);
        pcshow(targetPc);
        title('Target Point Cloud');
        subplot(1, 3, 2);
        pcshow(colorPc);
        title('Color Point Cloud');
        subplot(1, 3, 3);
        pcshow(colorPcTransformed);
        title('Transformed Color Point Cloud');
        drawnow();

        % ADDED PLOTTING
        figure
        pcshow(targetPc);
        hold on
        pcshow(colorPcTransformed);
        title('Two clouds superimposed');
        drawnow();

        % Perform IDW interpolation
        disp('Performing color interpolation...');
        interpolatedColors = interpolation(colorLocations, colorValues, targetLocations);

        % Ensure all colors are in the valid range [0, 255]
        interpolatedColors = max(0, min(255, interpolatedColors));
        interpolatedColors = uint8(interpolatedColors);

        % Create a new point cloud with the interpolated colors
        mergedPointCloud = pointCloud(targetLocations, 'Color', interpolatedColors);

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
                    [vertices, ~] = read_off(fullPath);
                case '.obj'
                    [vertices, ~] = read_obj(fullPath);
                case '.stl'
                    [vertices, ~] = read_stl(fullPath);
                case '.dae'
                    [vertices, ~] = read_dae(fullPath);
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
    case 'Pre-Align'
        % Select multiple cloud files for alignment
        [files, path] = uigetfile({'*.ply', 'PLY Files (*.ply)'}, 'Select Clouds for Pre-Align', 'MultiSelect', 'on');
        if isequal(files, 0), return; end
        if ischar(files), files = {files}; end

        % Load and align clouds to their planes
        clouds = cell(1, numel(files));
        for i = 1:numel(files)
            fullPath = fullfile(path, files{i});
            cloud = pcread(fullPath);
            cloud = align_to_plane(cloud);
            clouds{i} = cloud;
        end

        % Select XYZ and color clouds
        [indx, tf] = listdlg('PromptString', {'Select the XYZ cloud:'}, ...
            'SelectionMode', 'single', 'ListString', files);
        if ~tf, return; end
        xyzIdx = indx;

        [indx, tf] = listdlg('PromptString', {'Select the Color cloud:'}, ...
            'SelectionMode', 'single', 'ListString', files);
        if ~tf, return; end
        colorIdx = indx;

        % Ensure selected clouds are not the same
        if xyzIdx == colorIdx
            errordlg('Please select different clouds for XYZ and Color.', 'Error');
            return;
        end

        % Start with clouds overlapping
        refCloud = clouds{xyzIdx};
        movingCloud = clouds{colorIdx};
        movingCloud = position_at_origin(movingCloud, refCloud);

        % Store clouds for further processing
        f.UserData.refCloud = refCloud;
        f.UserData.movingCloud = movingCloud;

        % Display clouds for manual alignment
        cla(ax);
        pcshow(refCloud, 'Parent', ax, 'MarkerSize', 50);
        hold on;
        pcshow(movingCloud, 'Parent', ax, 'MarkerSize', 50);
        hold off;
        title(ax, 'Manually Align Clouds and Click Done');

        % Add manual alignment controls
        add_manual_alignment_controls(f, ax);

    otherwise
        warning('Unknown option selected.');
end
end

% Sub-function: Align cloud to its plane
function cloud = align_to_plane(cloud)
    pcData = cloud.Location;
    
    % Compute centroid
    centroid = mean(pcData, 1);
    
    % Center the data
    centeredData = pcData - centroid;
    
    % Compute the singular value decomposition
    [~, ~, V] = svd(centeredData, 'econ');
    
    % The normal vector of the plane is the last column of V
    normal = V(:, 3);
    zAxis = [0 0 1];
    
    % Compute rotation axis and angle
    rotAxis = cross(normal, zAxis);
    rotAngle = acos(dot(normal, zAxis));
    
    % Normalize the rotation axis
    if norm(rotAxis) > 1e-6  % Avoid division by zero
        rotAxis = rotAxis / norm(rotAxis);
        
        % Rodrigues' rotation formula
        K = [0 -rotAxis(3) rotAxis(2);
             rotAxis(3) 0 -rotAxis(1);
            -rotAxis(2) rotAxis(1) 0];

        rotMatrix = eye(3) + sin(rotAngle) * K + (1 - cos(rotAngle)) * (K * K);
    else
        rotMatrix = eye(3); % No rotation needed if already aligned
    end

    % Rotate all points in the point cloud
    alignedData = (rotMatrix * centeredData')';
    
    % Create the rotated point cloud, preserving color
    cloud = pointCloud(alignedData, 'Color', cloud.Color);
end

% Sub-function: Position moving cloud at the origin of the reference cloud
function cloud = position_at_origin(cloud, refCloud)
refCentroid = mean(refCloud.Location, 1);
cloudCentroid = mean(cloud.Location, 1);
translation = refCentroid - cloudCentroid;
translatedLocations = cloud.Location + translation;
cloud = pointCloud(translatedLocations, 'Color', cloud.Color);
end

% Sub-function: Add manual alignment controls
function add_manual_alignment_controls(f, ax)
% Add flip button
uicontrol(f, 'Style', 'pushbutton', 'String', 'Flip Cloud', ...
    'Position', [10, 10, 100, 30], ...
    'Callback', @(src, event) flip_current_cloud(f, ax));

% Add rotation and translation sliders
uicontrol(f, 'Style', 'text', 'Position', [150, 10, 80, 20], 'String', 'Rotate X');
uicontrol(f, 'Style', 'slider', 'Position', [230, 10, 200, 20], ...
    'Min', -180, 'Max', 180, 'Value', 0, ...
    'Callback', @(src, event) rotate_cloud(f, ax, 'x', src.Value));

uicontrol(f, 'Style', 'text', 'Position', [450, 10, 80, 20], 'String', 'Translate X');
uicontrol(f, 'Style', 'slider', 'Position', [530, 10, 200, 20], ...
    'Min', -0.5, 'Max', 0.5, 'Value', 0, ...
    'Callback', @(src, event) translate_cloud(f, ax, 'x', src.Value));

% Add a "Done" button
uicontrol(f, 'Style', 'pushbutton', 'String', 'Done', ...
    'Position', [750, 10, 80, 30], ...
    'Callback', @(src, event) finalize_alignment(f, ax));
end

% Sub-function: Flip cloud
function flip_current_cloud(f, ax)
movingCloud = f.UserData.movingCloud;
flipAxis = questdlg('Select Flip Axis', 'Flip Cloud', 'X', 'Y', 'Z', 'X');
if isempty(flipAxis), return; end
flippedLocations = movingCloud.Location;
switch flipAxis
    case 'X', flippedLocations(:, 1) = -flippedLocations(:, 1);
    case 'Y', flippedLocations(:, 2) = -flippedLocations(:, 2);
    case 'Z', flippedLocations(:, 3) = -flippedLocations(:, 3);
end
f.UserData.movingCloud = pointCloud(flippedLocations, 'Color', movingCloud.Color);

cla(ax);
pcshow(f.UserData.refCloud, 'Parent', ax, 'MarkerSize', 50);
hold on;
pcshow(f.UserData.movingCloud, 'Parent', ax, 'MarkerSize', 50);
hold off;
end

% Sub-function: Rotate cloud
function rotate_cloud(f, ax, axis, newValue)
if ~isfield(f.UserData, 'lastRotationValue')
    f.UserData.lastRotationValue = struct('x', 0, 'y', 0, 'z', 0);
end
% Retrieve the last rotation value for the given axis
lastValue = f.UserData.lastRotationValue.(axis);
deltaAngle = newValue - lastValue;
f.UserData.lastRotationValue.(axis) = newValue;

% Retrieve the current moving cloud
movingCloud = f.UserData.movingCloud;
% Compute the rotation matrix based on the specified axis
switch axis
    case 'x', R = eul2rotm([deg2rad(deltaAngle), 0, 0]);
    case 'y', R = eul2rotm([0, deg2rad(deltaAngle), 0]);
    case 'z', R = eul2rotm([0, 0, deg2rad(deltaAngle)]);
end
% Apply the rotation
rotatedLocations = (R * movingCloud.Location')';
f.UserData.movingCloud = pointCloud(rotatedLocations, 'Color', movingCloud.Color);

% Redisplay the updated clouds
cla(ax);
pcshow(f.UserData.refCloud, 'Parent', ax, 'MarkerSize', 50);
hold on;
pcshow(f.UserData.movingCloud, 'Parent', ax, 'MarkerSize', 50);
hold off;
end

% Sub-function: Translate cloud
function translate_cloud(f, ax, axis, newValue)
if ~isfield(f.UserData, 'lastTranslationValue')
    f.UserData.lastTranslationValue = struct('x', 0, 'y', 0, 'z', 0);
end
% Retrieve the last translation value for the given axis
lastValue = f.UserData.lastTranslationValue.(axis);
deltaValue = newValue - lastValue;
f.UserData.lastTranslationValue.(axis) = newValue;

% Retrieve the current moving cloud
movingCloud = f.UserData.movingCloud;
% Apply the translation to the specified axis
translatedLocations = movingCloud.Location;
switch axis
    case 'x', translatedLocations(:, 1) = translatedLocations(:, 1) + deltaValue;
    case 'y', translatedLocations(:, 2) = translatedLocations(:, 2) + deltaValue;
    case 'z', translatedLocations(:, 3) = translatedLocations(:, 3) + deltaValue;
end
f.UserData.movingCloud = pointCloud(translatedLocations, 'Color', movingCloud.Color);

% Redisplay the updated clouds
cla(ax);
pcshow(f.UserData.refCloud, 'Parent', ax, 'MarkerSize', 50);
hold on;
pcshow(f.UserData.movingCloud, 'Parent', ax, 'MarkerSize', 50);
hold off;
end

% Sub-function: Finalize alignment
function finalize_alignment(f, ax)
% Retrieve reference and moving clouds
refCloud = f.UserData.refCloud;
movingCloud = f.UserData.movingCloud;

% Perform rigid registration to align the moving cloud to the reference
tform = pcregrigid(movingCloud, refCloud, 'Metric', 'pointToPoint', ...
    'Tolerance', [0.001, 0.005], 'MaxIterations', 50);

% Transform the moving cloud based on the calculated transformation
alignedCloud = pctransform(movingCloud, tform);

% Store the aligned cloud in UserData for further use
f.UserData.pcData = alignedCloud;

% Enable save options after successful alignment
f.UserData.saveAllMenu.Enable = 'on';
f.UserData.saveBrushedMenu.Enable = 'on';

% Display the aligned cloud on the axes
cla(ax);
pcshow(alignedCloud, 'Parent', ax, 'MarkerSize', 50);
title(ax, 'Final Aligned Cloud');

% Display a message indicating successful alignment
disp('Clouds aligned successfully. Final transformation applied.');
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
[stl_data, ~, ~] = stlread(filename); % Use MATLAB's built-in stlread function
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

function interpolatedColors = interpolation(colorLocations, colorValues, targetLocations)
% Perform interpolation
tic
numPoints = size(targetLocations, 1);
interpolatedColors = zeros(numPoints, 3);
num_avg = 8;
for i = 1:numPoints
    distances = sqrt(sum((colorLocations - targetLocations(i,:)).^2, 2));
    % [~,idx] = sort(distances);
    % nearlist = idx(1:num_avg);
    % interpolatedColors(i, :) = sum(colorValues(nearlist,:)) / num_avg;
    [~,idx] = sort(distances);
    interpolatedColors(i, :) = sum(colorValues(idx(1:num_avg),:)) / num_avg;
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
