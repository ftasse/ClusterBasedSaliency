#include <fstream>
#include <string>
#include <regex>
#include <sstream>

#include <boost/filesystem.hpp>

#include "utils/cmd_parser.h"
#include "io/psb_classification.h"

namespace bf = boost::filesystem;

int main (int argc, char** argv) {
  // Parse cmd flags
  std::map<std::string, std::string> flags;
  parseCmdArgs(argc, argv, &flags);
  bool arg_error = false;

  std::string model_filepattern, cla_path, cla_prefix;
  std::string output_filelist;
  bool is_recursive;
  int cla_increment;
  if (!getFlag(flags, "model_filepattern", "", &model_filepattern)) {
    printf("Error: specify --model_filepattern \n");
    arg_error = true;
  }
  if (!getFlag(flags, "output_filelist", "", &output_filelist)) {
    printf("Error: specify --output_filelist \n");
    arg_error = true;
  }
  getFlag(flags, "cla", "", &cla_path);
  getFlag(flags, "cla_prefix", "", &cla_prefix);
  getFlag(flags, "cla_increment", 0, &cla_increment);
  getFlag(flags, "recursive", false, &is_recursive);
  if (arg_error) return 1;

  std::map<std::string, std::string> filenames;
  std::map<std::string, int> filename_occurences;

  std::string root_dir_path = "./";
  std::string dir_pattern = "", file_pattern = "*";
  size_t first_star = model_filepattern.find_first_of("*");
  size_t last_slash = model_filepattern.find_last_of("\\/");
  std::string slash = "";
  if (first_star == std::string::npos) first_star = model_filepattern.size();
  size_t last_slash_before_star =  
    model_filepattern.substr(0, first_star).find_last_of("\\/");
  if (last_slash_before_star != std::string::npos) {
    root_dir_path = model_filepattern.substr(0, last_slash_before_star+1);
    if (last_slash_before_star + 1 < model_filepattern.size()) {
      int dir_pattern_size = last_slash - (last_slash_before_star+1);
      if (dir_pattern_size > 0) {
        dir_pattern = model_filepattern.substr(last_slash_before_star+1, dir_pattern_size);
        slash += model_filepattern[last_slash];
      }
    }
  } 
  if (last_slash != std::string::npos && last_slash + 1 < model_filepattern.size()) {
    file_pattern = model_filepattern.substr(last_slash+1);
  }
  printf("decomposition: %s %s %s\n", 
    root_dir_path.c_str(), dir_pattern.c_str(), file_pattern.c_str());

  for (size_t i = 0; i < dir_pattern.size(); ++i) {
    if (dir_pattern[i] == '*') {
      dir_pattern.insert(i, ".");
      ++i;
    }
  }
  for (size_t i = 0; i < file_pattern.size(); ++i) {
    if (file_pattern[i] == '*') {
      file_pattern.insert(i, ".");
      ++i;
    }
  }

  bf::path fs_path = 
    bf::system_complete(bf::path(root_dir_path));
  const std::regex file_filter(fs_path.string()+dir_pattern+slash+file_pattern);
  const std::regex dir_filter(fs_path.string()+dir_pattern);
  printf("file pattern: %s\n", (fs_path.string()+dir_pattern+slash+file_pattern).c_str());
  printf("dir pattern: %s\n", (fs_path.string()+dir_pattern).c_str());

  bf::recursive_directory_iterator end_iter;
  for (bf::recursive_directory_iterator dir_itr(fs_path);
       dir_itr != end_iter; ++dir_itr) {
    if(!bf::is_regular_file( dir_itr->status() ) ) {
      if (!is_recursive)  dir_itr.no_push();
    } else {
      if( !std::regex_match(dir_itr->path().string(), file_filter)) continue; 
      std::string dir, basename;
      std::string file_path = dir_itr->path().string();
      get_dir_and_basename(file_path, dir, basename);
      if( !std::regex_match(dir, dir_filter)) continue;
      std::map<std::string, int>::iterator filename_occurences_it;
      filename_occurences_it = filename_occurences.find(basename);
      if (filename_occurences_it != filename_occurences.end()) {
        std::stringstream ss;
        ss << basename << "_" << filename_occurences_it->second;
        basename = ss.str();
        filename_occurences_it->second += 1;
      } else {
        filename_occurences.insert(make_pair(basename, 1));
      }
      filenames.insert(std::make_pair(basename, file_path));
    }
  }
  printf("Num of files: %lu\n", filenames.size()); 

  std::ofstream ofs(output_filelist.c_str());

  if (cla_path.size() > 0) {
    PSBClassification cla; cla.loadClassification(cla_path);
    std::vector<std::string> model_ids = cla.getAllSortedModelIds();
    for (int i = 0; i < model_ids.size(); ++i) {
      std::string key = model_ids[i];
      std::string::const_iterator key_it = key.begin();
      while (key_it != key.end() && std::isdigit(*key_it)) ++key_it;
      if (!key.empty() && key_it == key.end()) { // is a number
        int num = std::stoi (key,nullptr);
        std::stringstream ss;
        ss << cla_prefix << num+cla_increment;
        key = ss.str();
      }

      std::map<std::string, std::string>::iterator it = filenames.find(key);
      if (it != filenames.end())  {
        ofs << model_ids[i] << " " << it->second << "\n";
      } else {
        printf("Could not find filename corresponding to key: %s, id: %s\n", 
          key.c_str(), model_ids[i].c_str());
        ofs << model_ids[i] << " " << "null" << "\n";
      }
    }
  } else {
    std::map<std::string, std::string>::iterator filenames_it = filenames.begin();
    for (; filenames_it != filenames.end(); ++filenames_it) {
      ofs << filenames_it->first << " " << filenames_it->second << "\n";
    }
  }
  ofs.close();
  return 0;
}