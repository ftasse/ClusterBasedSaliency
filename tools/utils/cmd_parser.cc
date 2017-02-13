#include <stdio.h>
#include <sstream>
#include <fstream>

#include "cmd_parser.h"

using namespace std;

void parseCmdArgs(int argc, char **argv, 
		  std::map<string, string>* flags,
		  std::vector<string>* arguments,
		  bool verbose) {
  flags->clear();
  if (arguments) arguments->clear();

  string cur_flag = "";
  for (unsigned int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if (arg.size() > 2 && arg.substr(0, 2) == "--") {
      cur_flag = "";
      for (size_t k = 2; k < arg.size(); ++k) {
	if (arg[k] == '=') {
	  (*flags)[cur_flag] = arg.substr(k+1, arg.size() - (k+1));
	  cur_flag  = "";
	  break;
	}
	cur_flag += arg[k];
      }
    } else if (cur_flag.size() > 0) {
      (*flags)[cur_flag] = arg;
      cur_flag = "";
    } else if (arguments){
      arguments->push_back(arg);
    }
  }
  if (verbose) {
    if (flags->size() > 0) {
      printf("Flags\n-----\n");
      for (std::map<string, string>::iterator it = flags->begin();
	   it != flags->end(); ++it) {
	printf("%s: %s\n", it->first.c_str(), it->second.c_str());
      }
    }
    if (arguments && arguments->size() > 0) {
      printf("Args\n----\n");
      for (unsigned int i = 0; i < arguments->size(); ++i) {
	printf("arg %d: %s\n", i + 1, (*arguments)[i].c_str());
      }
    }
    printf("\n");
  }
}

bool getFlag(const std::map<std::string, std::string>& flags,
	     const string& name, const string& default_value, string* value) {
  value->clear();
  bool flag_set = false;
  std::map<string, string>::const_iterator it = flags.find(name);
  if (it != flags.end()) {
    *value = it->second;
    flag_set = true;
  } else {
    *value = default_value;
  }
  return flag_set;
}

template <typename T>
bool getFlag(const std::map<std::string, std::string>& flags,
	     const string& name, const T& default_value, T* value) {
  string str;
  bool flag_set = getFlag(flags, name, "", &str);
  if (!flag_set || !(std::istringstream (str.c_str()) >> (*value))) {
    *value = default_value;
  }
  return flag_set;
}

bool getMapFromFilelist(const std::string& filelist, std::map<std::string, std::string>& id_to_filename) {
  id_to_filename.clear();
  std::vector<std::pair<std::string, std::string> > pair_vector;
  if (!getMapFromFilelist(filelist, pair_vector))
    return false;
  for (int i = 0; i < pair_vector.size(); ++i)
    id_to_filename.insert(pair_vector[i]);
  return true;
}

bool getMapFromFilelist(
  const std::string& filelist, 
  std::vector<std::pair<std::string, std::string> >& id_to_filename) {
    id_to_filename.clear();
    std::ifstream filelist_in (filelist.c_str());
    if (!filelist_in.is_open()) return false;
    while (!filelist_in.eof()) {
      std::string fid, filename;
      filelist_in >> fid >> filename;
      if (fid.size() == 0 && filename.size() == 0) continue;
      else id_to_filename.push_back(make_pair(fid, filename));
    }
    return true;
}

bool getMapFromFilename(const std::string& filename, std::map<std::string, std::string>& id_to_filename) {
  id_to_filename.clear();
  std::vector<std::pair<std::string, std::string> > pair_vector;
  if (!getMapFromFilename(filename, pair_vector))
    return false;
  for (int i = 0; i < pair_vector.size(); ++i)
    id_to_filename.insert(pair_vector[i]);
  return true;
}

bool getMapFromFilename(
    const std::string& filename, 
    std::vector<std::pair<std::string, std::string> >& id_to_filename) {
  id_to_filename.clear();
  std::string dir, basename;
  get_dir_and_basename(filename, dir, basename);
  id_to_filename.push_back(make_pair(basename, filename));
  return true;
}

void get_dir_and_basename(const std::string path, std::string& dir, std::string& basename) {
  size_t sep = path.find_last_of("\\/");
    if (sep != std::string::npos) {
      basename = path.substr(sep + 1, path.size() - sep - 1);
      dir = path.substr(0, sep+1);
    }
    else {
      basename = path;
      dir = "./";
    }
    size_t dot = basename.find_last_of(".");
    if (dot != std::string::npos) basename = basename.substr(0, dot);
}

std::string generate_output_filename(
  std::string model_filename, std::string output_suffix, 
  std::string model_id, std::string output_prefix) {
  std::string model_dir, basename;
  get_dir_and_basename(model_filename, model_dir, basename);
  if (model_id.size() == 0) model_id = basename;
  if (output_prefix.size() == 0) output_prefix = model_dir;
  return output_prefix + model_id + output_suffix;
}

bool loadScalarFunction(const char* filename , std::vector<float>& values) {
  values.clear();
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
   return false;
 }
  std::string line;
  while (ifs >> line) {
    std::stringstream ss(line);
    float val;
    ss >> std::noskipws >> val;
    if (ss.eof() && !ss.fail())  values.push_back(val);
  }
  return true;
}


template bool getFlag<int>(const std::map<std::string, std::string>&,
			   const string&, const int&, int*);
template bool getFlag<size_t>(const std::map<std::string, std::string>&,
         const string&, const size_t&, size_t*);
template bool getFlag<float>(const std::map<std::string, std::string>&,
			     const string&, const float&, float*);
template bool getFlag<double>(const std::map<std::string, std::string>&,
			      const string&, const double&, double*);
template bool getFlag<bool>(const std::map<std::string, std::string>&,
            const string&, const bool&, bool*);