#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/encoder_module.h"
#include "modules/decoder_module.h"
#include "../utils/struct.h"
#include "../utils/utils.h"

void processFile(const std::filesystem::path &input_path, const std::string &output_dir, int q_level)
{
    std::cout << "\nProcessing file: " << input_path.string() << std::endl;

    // 1. Load point cloud
    std::vector<point_cloud> pcloud_data;
    std::string extension = input_path.extension().string();
    std::string filename = input_path.string();

    try
    {
        if (extension == ".bin")
        {
            load_pcloud(filename, pcloud_data);
        }
        else if (extension == ".ply")
        {
            load_pcloud_ply(filename, pcloud_data);
        }
        else
        {
            std::cerr << "Unsupported file format: " << extension << ". Skipping." << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading file " << filename << ": " << e.what() << std::endl;
        return;
    }

    if (pcloud_data.empty())
    {
        std::cerr << "Warning: No data loaded from " << filename << ". Skipping." << std::endl;
        return;
    }

    // 2. Encode Point Cloud
    EncoderModule encoder(4, q_level);
    std::vector<char> encoded_data = encoder.encodeToData(pcloud_data, true);

    // // 3. Save Compressed File
    // std::filesystem::path compressed_dir = std::filesystem::path(output_dir) / "compressed";
    // std::filesystem::create_directories(compressed_dir);

    // std::string compressed_filename = input_path.stem().string() + ".bin";
    // std::filesystem::path compressed_filepath = compressed_dir / compressed_filename;

    // std::cout << "  -> Saving compressed data to " << compressed_filepath.string() << std::endl;
    // std::ofstream compressed_file(compressed_filepath, std::ios::binary);
    // if (compressed_file.is_open()) {
    //     compressed_file.write(encoded_data.data(), encoded_data.size());
    //     compressed_file.close();
    // } else {
    //     std::cerr << "  Error: Could not open " << compressed_filepath.string() << " for writing." << std::endl;
    //     return;
    // }

    // 4. Decode point cloud
    std::cout << "  -> Decoding data..." << std::endl;
    DecoderModule decoder(encoded_data, 4, true, q_level);
    auto restored_pcloud = decoder.restored_pcloud;

    // Convert restored data to PCL format for saving
    pcl::PointCloud<pcl::PointXYZI>::Ptr restored_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    restored_pcl_cloud->width = restored_pcloud.size();
    restored_pcl_cloud->height = 1;
    restored_pcl_cloud->is_dense = true;
    restored_pcl_cloud->points.resize(restored_pcloud.size());

    for (size_t i = 0; i < restored_pcloud.size(); ++i)
    {
        restored_pcl_cloud->points[i].x = restored_pcloud[i].x;
        restored_pcl_cloud->points[i].y = restored_pcloud[i].y;
        restored_pcl_cloud->points[i].z = restored_pcloud[i].z;
        restored_pcl_cloud->points[i].intensity = restored_pcloud[i].r;
    }

    // 5. Save decompressed file
    if (extension == ".ply")
    {
        std::string decompressed_filename = input_path.stem().string() + ".ply";
        std::filesystem::path decompressed_filepath = std::filesystem::path(output_dir) / decompressed_filename;

        std::cout << "  -> Saving decompressed point cloud to " << decompressed_filepath.string() << std::endl;
        pcl::io::savePLYFileASCII(decompressed_filepath.string(), *restored_pcl_cloud);
    }
    else
    {
        std::string decompressed_filename = input_path.stem().string() + ".bin";
        std::filesystem::path decompressed_filepath = std::filesystem::path(output_dir) / decompressed_filename;

        pcloud2bin(decompressed_filepath.string(), restored_pcloud);
    }

    // 6. Report Compression Stats
    try
    {
        size_t original_size = std::filesystem::file_size(input_path);
        double compression_ratio = (encoded_data.size() > 0) ? static_cast<double>(original_size) / encoded_data.size() : 0;

        std::cout << "  Compression results:" << std::endl;
        std::cout << "    Original size: " << original_size << " bytes" << std::endl;
        std::cout << "    Compressed size: " << encoded_data.size() << " bytes" << std::endl;
        std::cout << "    Compression ratio: " << std::fixed << std::setprecision(2) << compression_ratio << ":1" << std::endl;
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "  Could not get original file size: " << e.what() << std::endl;
    }
}

int main(int argc, char **argv)
{
    // --- Argument Parsing ---
    std::string input_directory_str;
    std::string output_directory_str = "compressed_data"; // Default output directory
    int q_level = -1;

    std::vector<std::string> positional_args;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "-o")
        {
            if (i + 1 < argc)
            {                                     // Ensure there is a value after the -o flag
                output_directory_str = argv[++i]; // Assign the next argument as the output directory
            }
            else
            {
                std::cerr << "Error: -o option requires one argument." << std::endl;
                std::cerr << "Usage: " << argv[0] << " <input_directory> <q_level> [-o <output_directory>]" << std::endl;
                return -1;
            }
        }
        else
        {
            positional_args.push_back(arg);
        }
    }

    if (positional_args.size() != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_directory> <q_level> [-o <output_directory>]" << std::endl;
        return -1;
    }

    input_directory_str = positional_args[0];
    try
    {
        q_level = std::stoi(positional_args[1]);
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << "Error: Invalid q_level '" << positional_args[1] << "'. Must be an integer." << std::endl;
        return -1;
    }

    // --- Directory and File Processing ---
    std::filesystem::path input_dir(input_directory_str);
    if (!std::filesystem::is_directory(input_dir))
    {
        std::cerr << "Error: Input path is not a valid directory: " << input_directory_str << std::endl;
        return -1;
    }

    // Create the output directory if it doesn't exist
    try
    {
        std::filesystem::create_directories(output_directory_str);
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "Error creating output directory '" << output_directory_str << "': " << e.what() << std::endl;
        return -1;
    }

    std::cout << "Starting batch compression..." << std::endl;
    std::cout << "Input Directory: " << input_directory_str << std::endl;
    std::cout << "Output Directory: " << output_directory_str << std::endl;
    std::cout << "Quantization Level: " << q_level << std::endl;

    // Iterate over files in the input directory
    for (const auto &entry : std::filesystem::directory_iterator(input_dir))
    {
        if (entry.is_regular_file())
        {
            std::string extension = entry.path().extension().string();
            if (extension == ".ply" || extension == ".bin")
            {
                processFile(entry.path(), output_directory_str, q_level);
            }
        }
    }

    std::cout << "Batch compression finished." << std::endl;

    return 0;
}