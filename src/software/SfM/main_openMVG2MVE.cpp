// Copyright (c) 2015 Entrescan

// This file was adapted from
// src/software/SfM/main_ComputeSfM_DataColor.cpp,
// src/software/SfM/main_openMG2PMVS.cpp,
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/image/image.hpp"
#include "openMVG/numeric/numeric.h"
#include "openMVG/sfm/sfm.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <string>
#include <vector>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;

using namespace std;

void ExportContiguousViews(
  const SfM_Data & sfm_data,
  Hash_Map<IndexT, IndexT> & map_viewIdToContiguous,
  const string & sImageDir,
  const string & sCameraDir)
{
  C_Progress_display my_progress_bar( sfm_data.GetViews().size(), cout,
                                      "\nExport contiguous undistorted images\n" );
  Image<RGBColor> image, image_ud;
  for(Views::const_iterator iter = sfm_data.GetViews().begin(); iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar) {
    const View * view = iter->second.get();
    if (!sfm_data.IsPoseAndIntrinsicDefined(view)) { continue; }
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
    const IntrinsicBase * cam = iterIntrinsic->second.get();
    const Pinhole_Intrinsic_Radial_K3 * phi = dynamic_cast<Pinhole_Intrinsic_Radial_K3*>(iterIntrinsic->second.get());
    if ((!cam) || (!phi)) { continue; }

    // We have a valid view with a corresponding camera & pose

    // Since some views can have some missing poses,
    // we reindex the poses to ensure a contiguous pose list.
    map_viewIdToContiguous.insert(make_pair(view->id_view, map_viewIdToContiguous.size()));
    ostringstream os;
    os << setw(8) << setfill('0') << map_viewIdToContiguous[view->id_view];

    // Export as Projective Cameras
    ofstream file(stlplus::create_filespec(sCameraDir, os.str() ,"txt").c_str());
    vector<double> params = phi->getParams();
    for (int i = 0; i < 6; i++) {
      file << params[i];
      if (i < 5) { file << " "; }
    }
    file << '\n';
    const Pose3 pose = sfm_data.GetPoseOrDie(view);
    file << pose.rotation() << '\n';
    file << pose.translation().transpose() << '\n';
    file.close();
    

    // Export as undistorted image
    const string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
    const string dstImage = stlplus::create_filespec(sImageDir, os.str(),"jpg");
    if (cam->have_disto()) {
      // undistort the image and save it
      ReadImage( srcImage.c_str(), &image);
      UndistortImage(image, cam, image_ud, BLACK);
      WriteImage(dstImage.c_str(), image_ud);
    }
    else {
    // (no distortion)
      // copy the image if extension match
      if (stlplus::extension_part(srcImage) == "JPG" ||
        stlplus::extension_part(srcImage) == "jpg")
      { stlplus::file_copy(srcImage, dstImage); }
      else {
        ReadImage( srcImage.c_str(), &image);
        WriteImage( dstImage.c_str(), image);
      }
    }
  }
}


/// Find the color of the SfM_Data Landmarks/structure
void ColorizeTracks(
  const SfM_Data & sfm_data,
  Hash_Map<IndexT, IndexT> & map_viewIdToContiguous,
  vector<Vec3> & vec_3dPoints,
  vector<Vec3> & vec_tracksColor,
  vector<vector<IndexT> > & vec_vec_tracksViewsContiguous)
{
  // Colorize each track
  //  Start with the most representative image
  //    and iterate to provide a color to each 3D point

  C_Progress_display my_progress_bar(sfm_data.GetLandmarks().size(), cout,
                                     "\nCompute scene structure color\n");
  vec_tracksColor.resize(sfm_data.GetLandmarks().size());
  vec_3dPoints.resize(sfm_data.GetLandmarks().size());
  vec_vec_tracksViewsContiguous.resize(sfm_data.GetLandmarks().size());

  //Build a list of contiguous index for the trackIds
  map<IndexT, IndexT> trackIds_to_contiguousIndexes;
  IndexT cpt = 0;
  for (Landmarks::const_iterator it = sfm_data.GetLandmarks().begin(); it != sfm_data.GetLandmarks().end(); ++it, ++cpt) {
    trackIds_to_contiguousIndexes[it->first] = cpt;
    vec_3dPoints[cpt] = it->second.X;

    // Record contiguous views in which track appears.
    const Observations & obs = it->second.obs;
    for( Observations::const_iterator iterObs = obs.begin(); iterObs != obs.end(); ++iterObs) {
      const size_t viewId = iterObs->first;
      vec_vec_tracksViewsContiguous[cpt].push_back(map_viewIdToContiguous[viewId]);
    }
  }

  // The track list that will be colored (point removed during the process)
  set<IndexT> remainingTrackToColor;
  transform(sfm_data.GetLandmarks().begin(), sfm_data.GetLandmarks().end(), inserter(remainingTrackToColor, remainingTrackToColor.begin()), stl::RetrieveKey());

  while( !remainingTrackToColor.empty() ) {
    // Find the most representative image (for the remaining 3D points)
    //  a. Count the number of observation per view for each 3Dpoint Index
    //  b. Sort to find the most representative view index

    map<IndexT, IndexT> map_IndexCardinal; // ViewId, Cardinal
    for (set<IndexT>::const_iterator iterT = remainingTrackToColor.begin(); iterT != remainingTrackToColor.end(); ++iterT) {
      const size_t trackId = *iterT;
      const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;

      for( Observations::const_iterator iterObs = obs.begin(); iterObs != obs.end(); ++iterObs) {
        const size_t viewId = iterObs->first;
        if (map_IndexCardinal.find(viewId) == map_IndexCardinal.end())
        { map_IndexCardinal[viewId] = 1; }
        else
        { ++map_IndexCardinal[viewId]; }
      }
    }

    // Find the View index that is the most represented
    vector<IndexT> vec_cardinal;
    transform(map_IndexCardinal.begin(), map_IndexCardinal.end(), back_inserter(vec_cardinal), stl::RetrieveValue());
    using namespace stl::indexed_sort;
    vector< sort_index_packet_descend< IndexT, IndexT> > packet_vec(vec_cardinal.size());
    sort_index_helper(packet_vec, &vec_cardinal[0], 1);

    // First image index with the most of occurence
    map<IndexT, IndexT>::const_iterator iterTT = map_IndexCardinal.begin();
    advance(iterTT, packet_vec[0].index);
    const size_t view_index = iterTT->first;
    const View * view = sfm_data.GetViews().at(view_index).get();
    const string sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
    Image<RGBColor> image;
    ReadImage(sView_filename.c_str(), &image);

    // Iterate through the remaining track to color
    // - look if the current view is present to color the track
    set<IndexT> set_toRemove;
    for (set<IndexT>::const_iterator iterT = remainingTrackToColor.begin(); iterT != remainingTrackToColor.end(); ++iterT) {
      const size_t trackId = *iterT;
      const Observations & obs = sfm_data.GetLandmarks().at(trackId).obs;
      Observations::const_iterator it = obs.find(view_index);

      if (it != obs.end()) {
        // Color the track
        const Vec2 & pt = it->second.x;
        const RGBColor color = image(pt.y(), pt.x());

        vec_tracksColor[ trackIds_to_contiguousIndexes[trackId] ] = Vec3(color.r(), color.g(), color.b());
        set_toRemove.insert(trackId);
        ++my_progress_bar;
      }
    }
    // Remove colored track
    for (set<IndexT>::const_iterator iter = set_toRemove.begin(); iter != set_toRemove.end(); ++iter)
    { remainingTrackToColor.erase(*iter); }
  }
}

/// Export 3D point vector and camera position to PLY format
static bool exportToPly(
  const string & sFileName,
  const vector<Vec3> & vec_points,
  const vector<Vec3> & vec_coloredPoints,
  const vector<vector<IndexT> > & vec_vec_tracksViews)
{
  ofstream outfile;
  outfile.open(sFileName.c_str(), ios_base::out);

  outfile << "ply"
    << '\n' << "format ascii 1.0"
    << '\n' << "comment generated by OpenMVG2MVE"
    << '\n' << "element vertex " << vec_points.size()
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "property float confidence"
    << '\n' << "property list int int visibility"
    << '\n' << "end_header" << endl;

  for (size_t i=0; i < vec_points.size(); ++i)  {
    outfile << vec_points[i].transpose()
      << " " << vec_coloredPoints[i].transpose()
      << " " << -1.f
      << " " << vec_vec_tracksViews[i].size();
    for (size_t j=0; j < vec_vec_tracksViews[i].size(); ++j)  {
      outfile << " " << vec_vec_tracksViews[i][j];
    }
    outfile << "\n";
  }

  outfile.flush();
  bool bOk = outfile.good();
  outfile.close();
  return bOk;
}

// Convert from a SfM_Data format to another
int main(int argc, char **argv) {
  CmdLine cmd;

  string
    sSfM_Data_Filename_In,
    sOutDir = "obsolete_SfM_output",
    sImageDir = "images",
    sCameraDir = "cameras_disto",
    sCloudDir = "clouds",
    sOutputViewsDummy_Out = "views.txt",
    sOutputPLY_Out = "calib.ply";

  cmd.add(make_option('i', sSfM_Data_Filename_In, "sfmdata"));
  cmd.add(make_option('o', sOutDir, "outdir"));

  try {
      if (argc == 1) throw string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const string& s) {
      cerr
        << "Export .\n"
        << "Usage: " << argv[0] << '\n'
        << "[-i|--sfmdata] filename, the SfM_Data reconstructed scene file to convert\n"
        << "[-o|--outdir] path\n"
        << endl;

      cerr << s << endl;
      return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename_In, ESfM_Data(ALL))) {
    cerr << endl
      << "The input SfM_Data file \"" << sSfM_Data_Filename_In << "\" cannot be read." << endl;
    return EXIT_FAILURE;
  }

  // Compute directory and paths for output
  sImageDir = stlplus::folder_append_separator(sOutDir) + sImageDir;
  sCameraDir = stlplus::folder_append_separator(sOutDir) + sCameraDir;
  sCloudDir = stlplus::folder_append_separator(sOutDir) + sCloudDir;
  sOutputViewsDummy_Out = stlplus::folder_append_separator(sOutDir) + sOutputViewsDummy_Out;
  sOutputPLY_Out = stlplus::folder_append_separator(sCloudDir) + sOutputPLY_Out;

  // Create output dirs
  bool bOk = false;
  stlplus::folder_create( sOutDir );
  stlplus::folder_create( sImageDir );
  stlplus::folder_create( sCameraDir );
  stlplus::folder_create( sCloudDir );
  if (stlplus::folder_exists(sOutDir) && stlplus::folder_exists(sImageDir) && stlplus::folder_exists(sCameraDir) && stlplus::folder_exists(sCloudDir))
  { bOk = true; }
  else
  { cerr << "Cannot access all of the desired output directories" << endl; }

  ofstream dummy_views_file;
  dummy_views_file.open(sOutputViewsDummy_Out.c_str(), ios_base::out);
  dummy_views_file
    << "MVE checks for the presence of this file to determine that this directory" << endl
    << "contains OpenMVE data, but MVE ignores the contents of this file." << endl;
  dummy_views_file.close();

  // We use this hashmap to reindex the poses to ensure a contiguous pose list.
  Hash_Map<IndexT, IndexT> map_viewIdToContiguous;
  ExportContiguousViews(sfm_data, map_viewIdToContiguous, sImageDir, sCameraDir);

  // Compute the scene structure color
  vector<Vec3> vec_3dPoints, vec_tracksColor;
  vector<vector<IndexT> > vec_vec_tracksViewsContiguous;
  ColorizeTracks(sfm_data, map_viewIdToContiguous, vec_3dPoints, vec_tracksColor, vec_vec_tracksViewsContiguous);

  // Export the SfM_Data scene in the expected format
  if (exportToPly(sOutputPLY_Out, vec_3dPoints, vec_tracksColor, vec_vec_tracksViewsContiguous))
  { return EXIT_SUCCESS; }
  else
  { return EXIT_FAILURE; }
}

