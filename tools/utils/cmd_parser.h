#ifndef SRC_TOOLS_CMDPARSER_H
#define SRC_TOOLS_CMDPARSER_H

#include <map>
#include <string>
#include <vector>

void parseCmdArgs(int argc, char **argv, 
		  std::map<std::string, std::string>* flags,
		  std::vector<std::string>* arguments = NULL,
		  bool verbose = true);

bool getFlag(const std::map<std::string, std::string>& flags,
	     const std::string& name, const std::string& default_value, std::string* value);

template <typename T>
bool getFlag(const std::map<std::string, std::string>& flags,
	     const std::string& name, const T& default_value, T* value);

bool getMapFromFilelist(const std::string& filelist, std::map<std::string, std::string>& id_to_filename);
bool getMapFromFilename(const std::string& filename, std::map<std::string, std::string>& id_to_filename);

bool getMapFromFilelist(
  const std::string& filelist, 
  std::vector<std::pair<std::string, std::string> >& id_to_filename);

bool getMapFromFilename(
  const std::string& filename, 
  std::vector<std::pair<std::string, std::string> >& id_to_filename);

void get_dir_and_basename(
  const std::string path, std::string& dir, std::string& basename);

std::string generate_output_filename(
  std::string model_filename, std::string output_suffix, 
  std::string model_id = "", std::string output_prefix = "");

bool loadScalarFunction(const char* filename , std::vector<float>& values);

#endif // SRC_TOOLS_CMDPARSER_H
