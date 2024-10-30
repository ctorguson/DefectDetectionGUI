Overview

This MATLAB-based GUI enables users to load, visualize, and process point cloud data. The GUI includes tools for interactive 3D visualization, merging of multiple point clouds, meshing, densification, and flattening of surfaces, making it particularly useful for applications in 3D modeling, quality control, and visual inspection.
Features

    Point Cloud Loading: Supports, consumes and saves .ply files. 
    
    Merge Multiple: Merges multiple point clouds of the same object, aligns colors from one cloud onto another with precise interpolation. 
    
    Flattening Surface Fit: Flattens curved point clouds based on a third-order polynomial fit, offering options for outlier removal and re-fitting without outliers.
    
    Densification: Uses interpolation to increase point density within a cloud.
    
    3D Grid: Enhances visualization with customizable grid overlays.
    
    Depth: Maps the selected cloud by height data, useful for finding indentations. 
    
    Save Options: Saves point cloud data, with the option to save either all points or only those selected through brushing.

Future versions:

    Defect Detection: Highlights potential defect regions in the point cloud using color coding based on height values.

    Support multiple formats, including .ply, .las, .xyz, .pts, and .pcd. 

    Mesh Generation: Opens an existing mesh and generates and visualizes a mesh from a point cloud using Delaunay triangulation.
    
Installation
 
    Setup: Clone the repository and add .txt and .cpp files to a project in vscode or like software. Add .m files to MATLAB’s path, and run GUI(). 

Usage

    Save GUI() as .m file in MATLAB
    
    Run the GUI in matlab

    "Open" Dropdown Menu:
        Use the "Open" menu to select and load point cloud files. Available options include:
                "Open Cloud" (single file loading) 
                "Merge Multiple" (multi-cloud merging)
                "Merge Sections" (Stitch together peices of a cloud)
    
    "Visualization" Dropdown Menu: 
                "Grid" Makes a grid on the point cloud for inspection or brushing a smaller gridspace.
                "Densification" To make cloud points more dense. 
                "Defect" To locate possible defects in a part
                "Depth"
        
        Click within the axes to select specific points, which can be brushed for saving, resized, .
   
    Save Processed Data:
        Save all or selected points using the "Save Cloud As" menu.

Versioning

    Version 0 (2023-01-01): Basic outline and functionalities organized.

    Version 1 (2024-10-07): Initial beta release with basic functionalities.
        Fixed mis-scaled ATOS data, added figures for interim steps, included drawnow() for smoother figure updates.
    Version 2 (2024-10-30):
        Removed extra figure window, displayed merged cloud directly in main GUI window.

Dependencies

Dependencies: MATLAB R2022a and R2022b or later with Computer Vision Toolbox.
External Libraries: Ensure that Point Cloud Library (PCL) and all PCL dependencies and libraries are accessible - Necessary for extended functionality in some features.

# MatlabEnvironmentForGUI with PCL
arch -x86_64 
MATLAB_R2023b.app
Currently consumes .PLY files 
