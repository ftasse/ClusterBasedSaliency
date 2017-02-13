#ifndef PSB_CLASSIFICATION_H
#define PSB_CLASSIFICATION_H

#include <string>
#include <vector>
#include <map>
#include <memory>

class PSBClassification
{
public:
  class Category {
  public:
    Category(std::string title="", Category *parent=NULL);
    void print(std::string prefix="");

    std::string title() const { return title_; }
    std::string parent_title() const {
      if (!parent_) return "";
      else return parent_->title();
    }

    int count() const { return count_; }
    void set_count(int count);

    void addModel(std::string model_id) {
      model_ids_.push_back(model_id);
    }

    std::vector<std::string> model_ids() const {
      return model_ids_;
    }

    std::vector<std::string> subcategory_titles() const {
      std::vector<std::string> subcategory_titles(subcategories_.size());
      for (unsigned int i=0; i<subcategories_.size(); ++i)
        subcategory_titles[i] = subcategories_[i]->title();
      return subcategory_titles;
    }

  private:
    void addSubcategory(Category *subcategory);

    std::string title_;
    int count_;
    Category *parent_;
    std::vector<Category*> subcategories_;
    std::vector<std::string> model_ids_;
  };

public:
  PSBClassification();
  ~PSBClassification();

  std::string path() const { return path_; }

  bool loadClassification(std::string path);
  void print();

  std::vector<std::string> getModelCategoryTitles(std::string modelid) const;
  std::vector<std::string> getCategoryAncestorTitles(std::string title) const;

  std::pair<std::string, int> getCategoryInfo(std::string title) const;
  std::vector<std::string> getCategoryModels(std::string category_title) const;

  std::vector<std::string> getMainCategoryTitles() const;

  void getModelIndicesPerCategory(
    std::map<std::string, std::vector<int> >& category_model_ids) const ;

  std::string index_to_modelid(int pos) const {
    if (pos < 0 || pos >= index_to_modelid_.size())
      return "";
    return index_to_modelid_[pos];
  }

  int modelid_to_index (std::string id) const {
    std::map<std::string, int>::const_iterator it = modelid_to_index_.find(id);
    if (it == modelid_to_index_.end())
      return -1;
    return it->second;
  }

  std::vector<std::string> getAllSortedModelIds() const {
    return index_to_modelid_;
  }

  size_t num_models() const {
    return index_to_modelid_.size();
  }

private:
  void initCategoryTree();
  Category *getModelCategory(std::string modelid);

  std::string version_number_;
  std::string path_;
  std::map<std::string, Category* > categories_;
  std::map<std::string, std::string> model_category_titles_;

  std::vector<std::string> index_to_modelid_;
  std::map<std::string, int> modelid_to_index_;
};

#endif // PSB_CLASSIFICATION_H
