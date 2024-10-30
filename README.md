Overview

This MATLAB-based GUI enables users to load, visualize, and process point cloud data. The GUI includes tools for interactive 3D visualization, merging of multiple point clouds, meshing, densification, and flattening of surfaces, making it particularly useful for applications in 3D modeling, quality control, and visual inspection.
Features

    Point Cloud Loading: Supports multiple formats, including .ply, .las, .xyz, .pts, and .pcd. 
    Merging and Registration: Merges multiple point clouds, aligns colors from one cloud onto another with precise interpolation, and displays merged clouds within the main GUI         window.
    Flattening Surface Fit: Flattens curved point clouds based on a third-order polynomial fit, offering options for outlier removal and re-fitting without outliers.
    Densification: Uses interpolation to increase point density within a cloud.
    Mesh Generation: Generates and visualizes a mesh from the point cloud using Delaunay triangulation.
    Defect Detection: Highlights potential defect regions in the point cloud using color coding based on height values.
    3D Grid and Depth Controls: Enhances visualization with customizable grid overlays and depth display.
    Save Options: Saves point cloud data in multiple formats, with the option to save either all points or only those selected through brushing.

Installation

    Dependencies: MATLAB R2022a or later with Computer Vision Toolbox.
    External Libraries: Ensure that Point Cloud Library (PCL) and TIFF libraries are accessible if running in C++ mode.
    Setup: Clone the repository, add all files to MATLAB’s path, and run GUIMorse().

Usage

    Run the GUI:

    matlab

    GUIMorse();

    Load Point Clouds:
        Use the "Open" menu to select and load point cloud files.
        Available options include single file loading, multi-cloud merging, and mesh file loading.
    Visualize and Process:
        Toggle various visualization options like grid overlay, densification, defect highlighting, etc.
        Click within the axes to select specific points, which can be brushed for saving.
    Save Processed Data:
        Save all or selected points using the "Save Cloud As" menu.

Versioning

    Version 1 (2024-10-07): Initial beta release with basic functionalities.
        Fixed mis-scaled ATOS data, added figures for interim steps, included drawnow() for smoother figure updates.
    Version 2 (2024-10-30):
        Removed extra figure window, displayed merged cloud directly in main GUI window.

Dependencies

Ensure MATLAB R2022a or later and required toolboxes are installed. External libraries, such as Point Cloud Library (PCL) and TIFF, are recommended for extended functionality in some features.

# MatlabEnvironmentForGUI with PCL
arch -x86_64 
MATLAB_R2023b.app
Currently consumes .PLY files 
