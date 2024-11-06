Overview

This MATLAB-based GUI enables users to load, visualize, and process point cloud data. The GUI includes tools for interactive 3D visualization, merging of multiple point clouds, meshing, densification, and flattening of surfaces, making it particularly useful for applications in 3D modeling, quality control, and visual inspection.
Features

    Open: Point Cloud Loading Displays, consumes and saves .ply files. 
    
    Merge Multiple: Merges multiple point clouds of the same object, registers two clouds of the       same object to each other and interpolates colors from one cloud onto the other with precise       interpolation. 
    
    Flatten: Surface Fit Flattens curved point clouds based on a third-order polynomial fit,           offering options for outlier removal and re-fitting without outliers.
    
    Densifiy: Uses interpolation to increase point density within a cloud.
    
    Grid: Enhances visualization with customizable grid overlays.
    
    Depth: Maps the selected cloud by height data, useful for finding indentations. 

    Reset: Resets cloud to its original state upon opening. 
    
    Save Options: Saves point cloud data, with the option to save either all points or only those      selected through brushing.

Future versions:

    Defect: Highlights potential defect regions in the point cloud using color coding based on         height values.

    Support multiple formats, including .ply, .las, .xyz, .pts, and .pcd. 

    Mesh Generation: Opens an existing mesh and generates and visualizes a mesh from a point cloud     using Delaunay triangulation.
    
Installation
 
    Setup: Clone the repository and add .txt and .cpp files to a project in vscode or like             software. Add .m files to MATLAB’s path, and run GUI(). 

Usage

    Save GUI() as .m file in MATLAB
    
    Run the GUI in matlab

    "Open" Dropdown Menu:
        Use the "Open" menu to select and load point cloud file(s).
                
                "Open Cloud" (single file loading): This option is used to open a single                  file to apply visualization functions to. 
                
                 "Merge Multiple" (multi-cloud merging): This will open your file 
                 explorer where you will highlight the clouds you want to merge. You 
                 will need two clouds of the same section or part. If the clouds are not 
                 in the directory directly next to each other, you can highlight files 
                 in between them too. A prompt will then open and you will be asked to 
                 select the file you want to use for the xyz data. Select the file. A 
                 second prompt will open and you will be asked to select the file you
                 intend to use the color data from. Select the file. The program will 
                 then begin the process of reistering the two clouds to each other and 
                 interpolating the color data from the file chosen for color onto the 
                 xyz data of the file chosen for xyz data. 
                
                "Merge Segments" (Stitch together peices of a cloud): If two adjacent 
                 sections of a cloud have been separated, this function will stich them 
                 back together. This is useful for larger parts that need to be 
                 segmented. 
                 Processing may be time consuming and so in some cases you may want to     
                 work on sections individually and then stitch them back together after 
                 manipulation. 
    
    "Visualization" Dropdown Menu: 
                "Grid" Makes a grid on the point cloud for inspection or brushing a                                smaller gridspace. This feature allows the user to neatly break a part 
                into smaller sections. Simply apply grid to the part and use the brush 
                feature to brush and then the save brushed points feature to save that 
                section as a new file. These sections can be stiched back together later 
                using the Merge sections function. 
                
                "Densify" To make cloud points more dense. 
                
                "Defect" To locate possible defects in a part using SQUID, open the part 
                and select the "Defect" fuction.
                
                "Depth" To map the selected cloud by height data, useful for finding                               indentations. This is most useful when the cloud is first flattened or 
                laid flat on its axis usin the "Flat" or "Plane" functions

                "Plane" Adjusts the cloud so that it lays neatly on its plane if it is 
                not already positioned conveniently.

                "Flat" Lays out curved surfaces flat. 

                "Mesh" Turns a point cloud file into a mesh file. 

                "Reset" Returns cloud to its original state from the time of opening. 
                
        Click within the axes to use the tool bar: 
        "Brush" icon for brushing a section to save brushed for saving 
        "Plus" and "Minus" icons allow the cloud to be resized
        "Rotate" (A circle with an arrow) icon allows the part to be rotated in 3 
        dimesions
        "Pan" (A hand) icon allows the cloud to be slid around in the window
        "Axis" icon allows for the part to be flipped onto a different axis
        "Background Color" icon allows for the background color to be changed from the 
        default color (Black). This is helpful for visualizing parts of different colors
   
    Save Processed Data:
        Save all or selected points using the "Save Cloud As" menu.

Versioning

    Version 0 (2023-01-01): Basic outline and functionalities organized.

    Version 1 (2024-10-07): Initial beta release with basic functionalities.
        Fixed mis-scaled ATOS data, added figures for interim steps, included drawnow() for smoother figure updates.
    Version 2 (2024-10-30):
        Removed extra figure window, displayed merged cloud directly in main GUI window.

MATLAB Environment For GUI with PCL:
     arch -x86_64 
     MATLAB_R2023b.app
     Currently consumes .PLY files 

Dependencies:

MATLAB R2022a and R2022b or later with Computer Vision Toolbox.
External Libraries: Ensure that Point Cloud Library (PCL) and all PCL dependencies and libraries are accessible - Necessary for extended functionality in some features.
