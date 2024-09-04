#include <fstream>
#include <iostream>
#include <mex.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <sstream>
#include <string>
#include <vector>

typedef pcl::PointXYZRGBNormal PointType;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  mexPrintf("Starting PointProcessing...\n");

  // Check for the correct number of input arguments
  if (nrhs < 1) {
    // Updated error message ID for input requirement
    mexErrMsgIdAndTxt("MATLAB:PointProcessing:inputNotProvided",
                      "Input PLY file required.");
  }

  // Convert the input argument to a string
  char *inputFilePath = mxArrayToString(prhs[0]);
  if (inputFilePath == NULL) {
    // Error handling for conversion failure
    mexErrMsgIdAndTxt("MATLAB:PointProcessing:invalidInput",
                      "Failed to convert input to string.");
  }

  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);

  // Open the PLY file manually
  std::ifstream plyFile(inputFilePath);
  if (!plyFile.is_open()) {
    // Error message for file opening failure
    mexErrMsgIdAndTxt("MATLAB:PointProcessing:fileOpenFailed",
                      "Failed to open PLY file.");
  }

  std::string line;
  std::vector<std::string> properties;
  bool headerParsed = false;
  int vertexCount = 0;
  while (std::getline(plyFile, line)) {
    if (!headerParsed) {
      if (line.find("element vertex") != std::string::npos) {
        std::istringstream iss(line);
        std::string temp;
        iss >> temp >> temp >> vertexCount;
        mexPrintf("Detected vertex element with count: %d\n", vertexCount);
      } else if (line.find("property") != std::string::npos) {
        properties.push_back(line);
      } else if (line == "end_header") {
        headerParsed = true;
        mexPrintf("Header parsed successfully.\n");
        mexPrintf("Number of properties: %lu\n", properties.size());
        break;
      }
      continue;
    }
  }

  // Determine the indices for each property
  int xIndex = -1, yIndex = -1, zIndex = -1;
  int rIndex = -1, gIndex = -1, bIndex = -1;
  int nxIndex = -1, nyIndex = -1, nzIndex = -1;

  for (size_t i = 0; i < properties.size(); ++i) {
    if (properties[i].find(" x") != std::string::npos)
      xIndex = i;
    if (properties[i].find(" y") != std::string::npos)
      yIndex = i;
    if (properties[i].find(" z") != std::string::npos)
      zIndex = i;
    if (properties[i].find("red") != std::string::npos)
      rIndex = i;
    if (properties[i].find("green") != std::string::npos)
      gIndex = i;
    if (properties[i].find("blue") != std::string::npos)
      bIndex = i;
    if (properties[i].find("nx") != std::string::npos)
      nxIndex = i;
    if (properties[i].find("ny") != std::string::npos)
      nyIndex = i;
    if (properties[i].find("nz") != std::string::npos)
      nzIndex = i;
  }

  if (xIndex == -1 || yIndex == -1 || zIndex == -1 || rIndex == -1 ||
      gIndex == -1 || bIndex == -1 || nxIndex == -1 || nyIndex == -1 ||
      nzIndex == -1) {
    // Updated error message for missing fields
    mexErrMsgIdAndTxt("MATLAB:PointProcessing:missingFields",
                      "PLY header does not contain all required fields.");
  }

  // Parse the vertex data
  cloud->points.resize(vertexCount);
  for (int i = 0; i < vertexCount; ++i) {
    std::getline(plyFile, line);
    std::istringstream iss(line);
    std::vector<float> data(properties.size());
    for (size_t j = 0; j < properties.size(); ++j) {
      iss >> data[j];
    }

    PointType &point = cloud->points[i];
    point.x = data[xIndex];
    point.y = data[yIndex];
    point.z = data[zIndex];
    point.r = static_cast<uint8_t>(data[rIndex]);
    point.g = static_cast<uint8_t>(data[gIndex]);
    point.b = static_cast<uint8_t>(data[bIndex]);
    point.normal_x = data[nxIndex];
    point.normal_y = data[nyIndex];
    point.normal_z = data[nzIndex];
  }
  plyFile.close();

  cloud->width = vertexCount;
  cloud->height = 1;
  cloud->is_dense = true;

  mexPrintf("Manually loaded PLY file with %d points.\n", vertexCount);

  // Perform Poisson Surface Reconstruction
  pcl::Poisson<PointType> poisson;
  poisson.setDepth(8);
  pcl::PolygonMesh mesh;
  poisson.setInputCloud(cloud);
  poisson.reconstruct(mesh);

  // Check if the Poisson reconstruction succeeded
  if (mesh.polygons.empty()) {
    mexPrintf("Poisson reconstruction failed, attempting Greedy Projection "
              "Triangulation...\n");

    pcl::GreedyProjectionTriangulation<PointType> gp3;
    gp3.setSearchRadius(0.02);            // Decrease search radius
    gp3.setMu(2.0);                       // Adjust Mu
    gp3.setMaximumNearestNeighbors(150);  // Increase max neighbors
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud);
    gp3.reconstruct(mesh);

    if (mesh.polygons.empty()) {
      // Error message for failed mesh reconstruction
      mexErrMsgIdAndTxt(
          "MATLAB:PointProcessing:meshReconstructionFailed",
          "Mesh reconstruction failed or resulted in an empty mesh.");
    }
  }

  mexPrintf("Mesh reconstruction successful.\n");

  // Extract vertex colors and positions
  pcl::PointCloud<PointType>::Ptr vertices(new pcl::PointCloud<PointType>);
  pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

  // Manually map colors from original cloud to mesh vertices
  for (size_t i = 0; i < vertices->points.size(); ++i) {
    vertices->points[i].r = cloud->points[i].r;
    vertices->points[i].g = cloud->points[i].g;
    vertices->points[i].b = cloud->points[i].b;
  }

  // Check the first few vertices
  for (int i = 0; i < std::min(5, (int)vertices->points.size()); ++i) {
    const auto &pt = vertices->points[i];
    mexPrintf("Vertex %d: x=%f, y=%f, z=%f, r=%d, g=%d, b=%d\n", i, pt.x, pt.y,
              pt.z, pt.r, pt.g, pt.b);
  }

  mexPrintf("Vertices extracted from mesh.\n");

  // Output the mesh to an OFF file
  // Output the mesh to an OFF file in the 'build' directory
  std::ofstream out(
      "/Users/ciaratorguson/PointProcessing_PCL/build/output_mesh.off");
  if (!out) {
    // Handle file opening error
    mexErrMsgIdAndTxt("PCLforVSCode:FileOpenError",
                      "Unable to open output file.");
  }

  out << "OFF\n";
  out << vertices->points.size() << " " << mesh.polygons.size() << " 0\n";
  for (size_t i = 0; i < vertices->points.size(); ++i) {
    out << vertices->points[i].x << " " << vertices->points[i].y << " "
        << vertices->points[i].z << " "
        << static_cast<int>(vertices->points[i].r) << " "
        << static_cast<int>(vertices->points[i].g) << " "
        << static_cast<int>(vertices->points[i].b) << "\n";
  }
  for (const auto &polygon : mesh.polygons) {
    out << "3 " << polygon.vertices[0] << " " << polygon.vertices[1] << " "
        << polygon.vertices[2] << "\n";
  }
  out.close();

  mexPrintf("Mesh written to output_mesh.off.\n");

  // Clean up
  mxFree(inputFilePath);
}
