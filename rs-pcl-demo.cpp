// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering
#include <librealsense2/rsutil.h>  

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


#include "boost/date_time/posix_time/posix_time.hpp"
namespace pt = boost::posix_time;



using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);


pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}


float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}


int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense PCL Pointcloud Example");
    // Construct an object to manage view state
    //state app_state;
    glfw_state app_state;
    
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

	rs2::colorizer c;                     // Helper to colorize depth images
	texture renderer;                     // Helper for renderig images
	
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg; 

	// depth stream config  
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);  
  
	// colour stream config  
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30); 


    //Calling pipeline's start() without any additional parameters will start the first device
    // with its default streams.
    //The start function returns the pipeline profile which the pipeline used to start the device
    rs2::pipeline_profile profile = pipe.start(cfg);
    
    auto sensor = profile.get_device().first<rs2::depth_sensor>();  
        
	// Declare filters
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    
    
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 8); // 5-> 12288 , 6-> 8640
        
    // Enable hole-filling
	// Hole filling is an aggressive heuristic and it gets the depth wrong many times
	// However, this demo is not built to handle holes 
	// (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
	spat_filter.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    
    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    //float depth_scale = get_depth_scale(profile.get_device());
    float depth_scale = sensor.get_depth_scale();         //result: depthscale: 0.001

    
    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    
    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    
    rs2::align align(align_to);
       
	pcl_ptr pcl_points (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::visualization::PCLVisualizer  cloud_viewer_("Cloud viewer");
  
    cloud_viewer_.initCameraParameters();
    cloud_viewer_.setCameraPosition(0,0,0,0,0,1,0,-1,0);
    
    
    pcl_ptr downSizeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	
	
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	
	
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	
    //while (app) // Application still alive?
    while(!cloud_viewer_.wasStopped())
    {
		cloud_viewer_.removeAllPointClouds();
		// Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
		
		// rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
        // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
        //  after the call to wait_for_frames();
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
        {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
        }
   
 
		////////----start time------//////////
		pt::ptime start = pt::microsec_clock::local_time();
 
        //Get processed aligned frame
        auto processed = align.process(frames);
        
		// Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
        
        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }
              
        rs2::frame filtered = aligned_depth_frame;
        
        ///////----start filter------/////////////
        //pt::ptime start_filter = pt::microsec_clock::local_time();
      
		filtered = dec_filter.process(filtered);
		
		///////----end filter-----////////
		//pt::ptime end_filter = pt::microsec_clock::local_time();
		//pt::time_duration filter_diff = end_filter - start_filter;
		//cout << "Filter Time difference is " <<  filter_diff.total_milliseconds() << " milliSeconds" << endl;
		
		
		///////----start spat filter------/////////////
        //pt::ptime start_spat_filter = pt::microsec_clock::local_time();
       
        filtered = spat_filter.process(filtered);
       
        ///////----end filter-----////////
		//pt::ptime end_spat_filter = pt::microsec_clock::local_time();	
		//pt::time_duration filter_spat_diff = end_spat_filter - start_spat_filter;
		//cout << "Filter Time difference is " <<  filter_spat_diff.total_milliseconds() << " milliSeconds" << endl;
        
        // Passing both frames to remove_background so it will "strip" the background
        // NOTE: in this example, we alter the buffer of the other frame, instead of copying it and altering the copy
        //       This behavior is not recommended in real application since the other frame could be used elsewhere
        //remove_background(other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance);
        
        // Generate the pointcloud and texture mappings
        //points = pc.calculate(aligned_depth_frame);
		points = pc.calculate(filtered);

		//std::cout << "depthscale: " << depth_scale << "\n";
		//std::cout << "min range: " << min << "\n";

		std::cout << "points number: " << points.size() << "\n";

		auto pcl_points = points_to_pcl(points);
			
		sor.setInputCloud(pcl_points);
		sor.setLeafSize(0.1f, 0.1f, 0.1f);
		//sor.setLeafSize(0.15f, 0.15f, 0.15f); // 4000- 5000
		//sor.setLeafSize(0.18f, 0.18f, 0.18f); // 2000- 3500

		sor.filter(*downSizeCloud);
		
		tree->setInputCloud (downSizeCloud);
		n.setInputCloud (downSizeCloud);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);
		
		pcl::concatenateFields (*downSizeCloud, *normals, *cloud_with_normals);
		
		tree2->setInputCloud (cloud_with_normals);
		
		//gp3.setSearchRadius(1.0);   //search radius = 1 m
		gp3.setSearchRadius(0.2);		//search radius = 0.2 m

		// Set typical values for the parameters
		gp3.setMu (2.5);
		//gp3.setMaximumNearestNeighbors(80);
		gp3.setMaximumNearestNeighbors(20);


		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (triangles);
		
		std::cout << "--mesh --"<<triangles.polygons.size() << endl;
		
		
		////////----end time------//////////
		pt::ptime end = pt::microsec_clock::local_time();
		pt::time_duration diff = end - start;
		cout << "Time difference is " <<  diff.total_milliseconds() << " milliSeconds" << endl;
		
		cloud_viewer_.addPointCloud(pcl_points);
		cloud_viewer_.addPolygonMesh(triangles, "polygon");
		
		cloud_viewer_.spinOnce(100);

        // frame size  
		const int w_rgb = other_frame.as<rs2::video_frame>().get_width();  
		const int h_rgb = other_frame.as<rs2::video_frame>().get_height();  
		
		//std::cout << "RGB: w " << w_rgb << " \n";
		//std::cout << "RGB: h " << h_rgb << " \n";
		
		
		// frame size  
		const int w_depth = aligned_depth_frame.as<rs2::video_frame>().get_width();  
		const int h_depth = aligned_depth_frame.as<rs2::video_frame>().get_height();  
		//std::cout << "DEPTH: w " << w_depth << " \n";
		//std::cout << "DEPTH: h " << h_depth << " \n";
	/*
		        
        // Get the depth frame's dimensions
		float width = aligned_depth_frame.get_width();
		float height = aligned_depth_frame.get_height();

		// Query the distance from the camera to the object in the center of the image
		float dist_to_center = aligned_depth_frame.get_distance(width / 2, height / 2);

		// Print the distance
		std::cout << "The camera is facing an object " << dist_to_center << " meters away \n";
        
        
		//////////////////////////////////////////////////////////////////////////
		
		//auto color = frames.get_color_frame();
        //auto color = processed.get_color_frame();
		auto color = other_frame;
		
		
		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            //color = frames.get_infrared_frame();
			color = processed.get_infrared_frame();
        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);



		// Taking dimensions of the window for rendering purposes
        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());
        
         // At this point, "other_frame" is an altered frame, stripped form its background
        // Calculating the position to place the frame in the window
        rect altered_other_frame_rect{ 0, 0, w, h };
        altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });

        // Render aligned image
        //renderer.render(other_frame, altered_other_frame_rect);
        
        // The example also renders the depth frame, as a picture-in-picture
        // Calculating the position to place the depth frame in the window
        rect pip_stream_depth{ 0, 0, w / 5, h / 5 };
        pip_stream_depth = pip_stream_depth.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
        pip_stream_depth.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream_depth.w ;//- (std::max(w, h) / 25);
        pip_stream_depth.y = altered_other_frame_rect.y + altered_other_frame_rect.h - pip_stream_depth.h ;//- (std::max(w, h) / 25);

        // Render depth (as picture in pipcture)
        renderer.upload(c.process(aligned_depth_frame));
        renderer.show(pip_stream_depth);
        
        
        // The example also renders the depth frame, as a picture-in-picture
        // Calculating the position to place the color frame in the window
        rect pip_stream_color{ 0, 0, w / 5, h / 5 };
        pip_stream_color = pip_stream_color.adjust_ratio({ static_cast<float>(color.get_width()),static_cast<float>(color.get_height()) });
        pip_stream_color.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream_color.w ;//- (std::max(w, h) / 25);
        pip_stream_color.y = altered_other_frame_rect.y + altered_other_frame_rect.h - pip_stream_color.h- h/5;// - (std::max(w, h) / 25) ;
        
        // Render color (as picture in pipcture)
        renderer.upload(c.process(color));
        renderer.show(pip_stream_color);
        
        // Draw the pointcloud
        //draw_pointcloud(app.width(), app.height(), app_state, points);
        
        /////////////////////////////////////////////////////////////////////////
       
*/
	}


    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}




