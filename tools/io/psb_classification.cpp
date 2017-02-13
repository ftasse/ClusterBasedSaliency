#include "psb_classification.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>

bool compare_filenames(std::string a, std::string b)
{
  long a1 = -1, b1 = -1; char* p;
  for (size_t i = 0; i < a.size(); ++i) {
    if (isdigit(a[i])) {
      a1 = strtol(a.substr(i).c_str(), &p, 10);
      break;
    }
  }
  for (size_t i = 0; i < b.size(); ++i) {
    if (isdigit(b[i])) {
      b1 = strtol(b.substr(i).c_str(), &p, 10);
      break;
    }
  }
  // std::cout << a << " " << b << " "<< a1 << " " << b1 << "\n";
  
  if (a1 < b1)
    return true;
  if (a1 == b1)
    return a < b;
  return false;
}

PSBClassification::PSBClassification() {
}

PSBClassification::~PSBClassification() {
  for (std::map<std::string, Category*>::iterator mit = categories_.begin(); mit != categories_.end(); ++mit)
  {
    delete mit->second;
  }
}

void PSBClassification::initCategoryTree()
{
  categories_.clear();
  model_category_titles_.clear();
  categories_["0"] = (new Category ("0"));
}

bool PSBClassification::loadClassification(std::string path)
{
  std::ifstream ifs(path.c_str());
  if (!ifs.is_open()) {
    std::cerr << "Could not open file: " << path << "\n";
    std::cerr << std::flush;
    return false;
  }

  std::string format;
  ifs >> format;
  if (format.compare("PSB") != 0) {
    ifs.close();
    std::cerr << "The following format is not PSB: " << format << "\n";
    std::cerr << std::flush;
    return false;
  }

  ifs >> version_number_;
  if (version_number_.compare("1") != 0) {
    ifs.close();
    std::cerr << "The following version is not 1: " << version_number_ << "\n";
    std::cerr << std::flush;
    return false;
  }

  int num_categories, num_models;
  ifs >> num_categories >> num_models;

  initCategoryTree();
  path_ = path;

  for (int i=0; i<num_categories; ++i) {
    std::string category_title, parent_category_title;
    int num_category_models;
    ifs >> category_title >> parent_category_title >> num_category_models;

    if (categories_.find(category_title) != categories_.end())
      category_title += "*";

    Category *parent = categories_[parent_category_title];
    categories_[category_title] =  new Category (category_title, parent);

    Category *category = categories_[category_title];
    category->set_count(num_category_models);
    for (int j=0; j<num_category_models; ++j) {
       std::string model_id;
       ifs >> model_id;
       model_category_titles_[model_id] = category_title;
       category->addModel(model_id);
       // if (category_title == "Mouse") std::cout << category_title << "\t" << model_id << "\n";
    }
  }

  bool all_integers = true;
  std::vector<std::string> allmodels = getCategoryModels("0");

  for (unsigned int i = 0; i < allmodels.size(); i += 1)
  {
    std::string s = allmodels[i];
    bool is_integer = true;

    if(s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+')))
      is_integer = false ;
    else
    {
      char * p ;
      strtol(s.c_str(), &p, 10) ;
      is_integer = (*p == 0); 
    }

    if (!is_integer)
    {
      all_integers = false;
      break;
    }
  }

  // std::cout << "All integer ids? " << all_integers << "\n";

  if (!all_integers)
  {
    index_to_modelid_ = allmodels;
    std::sort(index_to_modelid_.begin(), index_to_modelid_.end(), compare_filenames);
    for (unsigned int i = 0; i < index_to_modelid_.size(); ++i) {
      // std::cout << i << " " << index_to_modelid_[i] << "\n";
      modelid_to_index_[index_to_modelid_[i]] = i;
    }
  } else
  {
    std::vector< std::pair<int, std::string> > model_indices(allmodels.size());
    for (unsigned int i = 0; i < allmodels.size(); i += 1)
      model_indices[i] = std::pair<int, std::string>(atoi(allmodels[i].c_str()), allmodels[i]);
    std::sort(model_indices.begin(), model_indices.end());

    index_to_modelid_.resize(allmodels.size());
    for (unsigned int i = 0; i < allmodels.size(); i += 1)
    {
      index_to_modelid_[i] = model_indices[i].second;
      modelid_to_index_[model_indices[i].second] = i;
    }
  }

  /*std::cout << "\n\n";
  for (unsigned int i = 0; i < allmodels.size(); i += 1)
    std::cout << index_to_modelid_[i] << " ";
  std::cout << "\n\n";*/

  // std::map<std::string, std::vector<int> > category_query_ids;
  // getModelIndicesPerCategory(category_query_ids); 
  // std::vector<int> ids = category_query_ids["Mouse"];
  // for (int i = 0; i < ids.size(); ++i)
  // {
  //   std::cout << i << " " << ids[i] << " " << index_to_modelid_[ids[i]] << "\n";
  // }

  return true;
}

void PSBClassification::print() {
  categories_["0"]->print();

  /*for (std::map<std::string, std::string>::iterator it = model_category_titles_.begin();
       it!= model_category_titles_.end(); ++it) {
    std::cout << it->first << " --> " << it->second << "\n";
  }*/
}

void PSBClassification::getModelIndicesPerCategory(
    std::map<std::string, std::vector<int> >& category_model_ids) const 
{
  category_model_ids.clear();
  std::vector<std::string> categories = getMainCategoryTitles();
  for (size_t i = 0; i < categories.size(); ++i) {
    category_model_ids[categories[i]] = std::vector<int>();
    std::pair<std::map<std::string, std::vector<int> >::iterator, bool> ret;
    ret = category_model_ids.insert(std::make_pair(categories[i], std::vector<int>()));
    if (ret.second != false) {
      std::cout << "This category has already being processed: " << categories[i] << "\n";
      continue;
    }

    std::vector<std::string> models = getCategoryModels(categories[i]);
    for (size_t j = 0; j < models.size(); ++j)
      ret.first->second.push_back(modelid_to_index(models[j]));
  }
}

PSBClassification::Category *PSBClassification::getModelCategory(std::string modelid) {
  std::map<std::string, std::string>::const_iterator mit;
  mit = model_category_titles_.find(modelid);
  if (mit == model_category_titles_.end())  return NULL;
  else
    return categories_[mit->second];
}

std::vector<std::string> PSBClassification::getModelCategoryTitles(std::string modelid) const {
  std::map<std::string, std::string>::const_iterator mit;
  mit = model_category_titles_.find(modelid);
  if (mit == model_category_titles_.end())
    return std::vector<std::string>();
  else {
      return getCategoryAncestorTitles(mit->second);
  }
}

std::vector<std::string> PSBClassification::getCategoryAncestorTitles(std::string title) const {
  std::vector<std::string> titles;
  while (title.size() > 0) {
      titles.push_back(title);
      title = categories_.find(title)->second->parent_title();
  }
  return titles;
}

std::pair<std::string, int> PSBClassification::getCategoryInfo(std::string title) const {
  std::map<std::string, Category*>::const_iterator cit;
  cit = categories_.find(title);
  return std::pair<std::string, int>(cit->second->title(), cit->second->count());
}

std::vector<std::string> PSBClassification::getCategoryModels(std::string category_title) const {
  std::vector<std::string> model_ids;
  std::map<std::string, Category*>::const_iterator cit;

  cit = categories_.find(category_title);
  if (cit == categories_.end())
    return model_ids;
  else {
      model_ids = cit->second->model_ids();
      std::vector<std::string> subcategory_titles = cit->second->subcategory_titles();
      for(unsigned int i=0; i<subcategory_titles.size(); ++i) {
          std::vector<std::string> ids = getCategoryModels(subcategory_titles[i]);
          model_ids.insert(model_ids.end(), ids.begin(), ids.end());
      }
      return model_ids;
  }
}

std::vector<std::string> PSBClassification::getMainCategoryTitles() const {
  std::map<std::string, Category*>::const_iterator cit;
  cit = categories_.find("0");
  return cit->second->subcategory_titles();
}

PSBClassification::Category::Category(std::string title, Category *parent)
  : title_(title), count_(0), parent_(parent){
  if (parent_)
    parent_->addSubcategory(this);
}

void PSBClassification::Category::addSubcategory(Category *subcategory) {
  subcategory->parent_ = this;
  subcategories_.push_back(subcategory);
  set_count(count() + subcategory->count());
}

void PSBClassification::Category::set_count(int count) {
  if (count != count_) {
      if (parent_)
        parent_->set_count(parent_->count()+count - count_);
      count_ = count;
    }
}

void PSBClassification::Category::print(std::string prefix) {
  std::cout << prefix << title() << ": " << count() << "\n";
  for (unsigned int i=0; i<subcategories_.size(); ++i) {
      subcategories_[i]->print(prefix+"-");
    }
}


