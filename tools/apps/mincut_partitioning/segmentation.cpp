
#include <map>
#include <queue>


#include <pcl/point_types.h>
#include "gco/GCoptimization.h"
#include "vlfeat/vl/gmm.h"
#include "mincut_partitioning/segmentation.h"

namespace Eigen {
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajorMatrixXf;
}

/*void calc_edge_min_weights(Mesh &mesh, Mesh::HalfedgeHandle heh, double &ang, double &len)
{
    double theta = mesh.calc_dihedral_angle(heh);
    // double weight =std::abs(theta/M_PI);
    // if ( theta > 0.0 ) weight*=5;

    double convexFac = 1;
    if (theta > 0.0)
        convexFac = 5;
    double weight = pow(1 - (1-cos(theta))*convexFac, 2);

    ang = weight;
    len = mesh.calc_edge_length(heh);
}*/

bool mincut(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
            const std::vector<std::vector<size_t> >& neighbours,
            const std::vector<std::vector<float> >& neighbours_weights,
            std::vector<int>& region, const Eigen::MatrixXf& samples,
            std::vector<int>& labels, int num_labels, double lambda, double alpha) {
    if (region.size() <= (unsigned int)10*num_labels)
        return false;
    int num_values = region.size();

    int max_iters = 30;
    float epsilon = 1e-1;
    Eigen::MatrixXf cur_samples = samples;
    if (region.size() != samples.rows()) {
        cur_samples =  Eigen::MatrixXf::Zero(region.size(), samples.cols());
        for (int i=0; i<cur_samples.rows(); ++i)
            for (int j=0; j<cur_samples.cols(); ++j)
                cur_samples(i, j) = samples(region[i], j);
    }
    Eigen::RowMajorMatrixXf rowmajor_cur_samples = cur_samples;
    float* sample_data = rowmajor_cur_samples.data();
    int dim = cur_samples.cols();
    int num_data = cur_samples.rows();
    
    // printf("data/dim/labels: %d %d %d %f %f \n", num_data, dim, num_labels, cur_samples.minCoeff(), cur_samples.maxCoeff());
    VlGMM* gmm = vl_gmm_new (VL_TYPE_FLOAT, dim, num_labels) ;
    vl_gmm_set_verbosity(gmm, 0);
    vl_gmm_set_max_num_iterations (gmm, max_iters) ;
    VlKMeans * kmeans = vl_kmeans_new (VL_TYPE_FLOAT, VlDistanceL2) ; 
    vl_kmeans_set_verbosity(kmeans, 0);
    vl_kmeans_set_algorithm (kmeans, VlKMeansANN) ; // VlKMeansANN or VlKMeansElkan or VlKMeansLloyd
    vl_kmeans_set_initialization (kmeans, VlKMeansPlusPlus); // VlKMeansPlusPlus or VlKMeansRandomSelection
    vl_gmm_set_initialization (gmm,VlGMMKMeans);
    vl_gmm_set_kmeans_init_object (gmm, kmeans);
    vl_gmm_cluster (gmm, sample_data, num_data);
    num_labels = vl_gmm_get_num_clusters(gmm);
    Eigen::RowMajorMatrixXf posteriors = Eigen::Map<Eigen::RowMajorMatrixXf>(
        (float*) vl_gmm_get_posteriors(gmm), cur_samples.rows(), num_labels);

    labels.clear(); labels.resize(region.size());
    for (size_t k = 0; k < labels.size(); ++k) {
        int label;
        posteriors.row(k).maxCoeff(&label);
        labels[k] = label;
    }

    int *data = new int[num_values*num_labels];
    for ( int i = 0; i < num_values; i++ )
        for (int l = 0; l < num_labels; l++ ) {
            data[i*num_labels + l] = -log(std::max(posteriors(i, l) , (float)1e-8));
            //if (gmmfit.probability(i, l) != 1 && gmmfit.probability(i, l) != 0) std::cout << "Error: " << i << " " << l << " "<< cur_samples.row(i) <<" " << gmmfit.probability(i, l) << "\n";
            //std::cout <<gmmfit.face_probs[i][l] << " " << -log(gmmfit.face_probs[i][l] + 1e-8)<< "\n" << std::flush;
        }
    int *smooth = new int[num_labels*num_labels];
    for ( int l1 = 0; l1 < num_labels; l1++ )
        for (int l2 = 0; l2 < num_labels; l2++ )
            smooth[l1+l2*num_labels] = (l1==l2)? 0:lambda;

    vl_gmm_delete (gmm);
    vl_kmeans_delete (kmeans);


    try {
        GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_values,num_labels);
        gc->setDataCost(data);
        gc->setSmoothCost(smooth);

        std::map<int, int> id_map;
        for (unsigned int i=0; i<region.size(); ++i) id_map[region[i]] = i;

        std::map<std::pair<int, int>, std::pair<double, double> > edgeinfo;  //Angle & weights
        std::map<std::pair<int, int>, std::pair<double, double> >::iterator it;
        double avg_weight = 0.0, avg_len = 0.0;

        for (unsigned int i=0; i<region.size(); ++i) {
            int idx = region[i];
            int count = 0;
            for (size_t c = 0; c < neighbours[idx].size(); ++c) {
                int other_idx = neighbours[idx][c];
                if (idx == other_idx || other_idx < 0 || id_map.find(other_idx) == id_map.end())
                    continue;

                double weight = neighbours_weights[idx][c];
                double len = (points->points[idx].getVector3fMap() - 
                              points->points[other_idx].getVector3fMap()).norm();
                avg_weight += weight;  avg_len += len;
                // avg_len = std::max(avg_len, len);
                // avg_weight = std::max(avg_weight, weight); avg_len = std::max(avg_len, len);
                int j = id_map[other_idx];
                if (i < j) edgeinfo[std::make_pair(i, j)] = (std::pair<double, double>(weight, len));
                else edgeinfo[std::make_pair(j ,i)] = (std::pair<double, double>(weight, len));
                ++count;
            }
        }

        // if (edgeinfo.size() > 0) {
        //     avg_weight /= edgeinfo.size();
        //     avg_len /= edgeinfo.size();
        // }
        for (it = edgeinfo.begin(); it != edgeinfo.end(); ++it) {
            // double weight = alpha*(it->second.first/avg_weight) + 
            //                (1-alpha)*(it->second.second/avg_len);
            double weight =  (it->second.second)*it->second.first/(avg_weight/edgeinfo.size());

            if (weight < 0.0 || weight > 1.0) std::cout << weight << "\n" << std::flush;
            gc->setNeighbors(it->first.first, it->first.second,  -log(std::max(weight, 1e-8)));
        }

        printf("Before optimization energy is %lld",gc->compute_energy()); std::cout << std::flush;
        gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
        printf("\nAfter optimization energy is %lld\n",gc->compute_energy()); std::cout << std::flush;
        for ( int  i = 0; i < num_values; i++ ) labels[i] = gc->whatLabel(i);
        delete gc;
    } catch (GCException e){
        e.Report();
    }

    delete [] smooth;
    delete [] data;

    bool segmented = false;
    int first_label = labels[0];
    for (unsigned int i=0; i<labels.size(); ++i)
        if (labels[i] != first_label) {
            segmented = true;   break;
        }
    std::cout << "segmented? " << segmented << "\n";
    return segmented;
}

void segmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr points,
                  const std::vector<std::vector<size_t> >& neighbours,
                  const std::vector<std::vector<float> >& neighbours_weights,
                  const Eigen::MatrixXf& samples,
                  std::vector< std::set<int> >& labelling, int num_labels,
                  bool recursive, bool hierarchical,
                  double lambda, double alpha) {
    int n_labels = num_labels;
    int n_values = points->size();

    labelling.clear();
    labelling.reserve(n_labels);
    std::queue<int> segmentids;

    std::vector<int> removed;

    do
    {
        int sid = -1;
        std::vector<int> segment;

        if (segmentids.empty())
        {
            segment.reserve(n_values);
            for (int i=0; i< n_values; ++i)
                segment.push_back(i);
        } else
        {
            sid = segmentids.front(); 
            segment.insert(segment.end(), labelling[sid].begin(), labelling[sid].end());
            segmentids.pop();
            std::cout << "\nResegment: " << sid << " " << segment.size() << "\n";
        }

        std::vector<int> labels;

        if (mincut(points, neighbours, neighbours_weights, segment, 
                   samples, labels, num_labels, lambda, alpha))
        {
            if (sid >=0)    removed.push_back(sid);
            std::map<int, std::set<int> > result;
            for (unsigned int j=0; j<labels.size(); ++j)
                result[labels[j]].insert(segment[j]);

            for (std::map<int, std::set<int> >::iterator mit = result.begin(); mit != result.end(); ++mit)
            {
                segmentids.push(labelling.size());
                labelling.push_back(mit->second);
            }
            // if (sid >=3) break;
        }

    }   while (recursive && !segmentids.empty());


    if (!hierarchical)
        for (int i = (int)removed.size()-1; i>=0; --i)
            labelling.erase(labelling.begin() + removed[i]);
}

/*
GMMFit::GMMFit(std::vector<double>& weights, int n_labels, bool _use_faces)
{
    use_faces = _use_faces;
    num_labels = n_labels;
    samples = new Eigen::MatrixXf(weights.size(), 1);
    for (unsigned int i=0; i<weights.size(); ++i)
        (*samples)(i, 0) = weights[i];
}

GMMFit::GMMFit(std::vector< std::vector<double> >& weights, int n_labels, bool _use_faces)
{
    use_faces = _use_faces;
    num_labels = n_labels;
    samples = new Eigen::MatrixXf(weights.size(), weights[0].size());
    for (unsigned int i=0; i<weights.size(); ++i)
    {
        for (unsigned int j=0; j<weights[i].size(); ++j)
            (*samples)(i, j) = weights[i][j];
    }
}

GMMFit::~GMMFit()
{
    delete samples;
}

void GMMFit::train(std::vector<int> mask_ids) {
    std::cout << "Mask size: " << mask_ids.size() << "\n" << std::flush;
    int max_iters = 30;
    float epsilon = 1e-6;

    Eigen::MatrixXi cvlabels = Eigen::MatrixXi::Zero(mask_ids.size(), 1);
    Eigen::MatrixXf cvprobs = Eigen::MatrixXf::Zero(mask_ids.size(), num_labels);
    Eigen::MatrixXf cur_samples = *samples;

    if (mask_ids.size() != samples->rows())
    {
        cur_samples =  Eigen::MatrixXf::Zero(mask_ids.size(), samples->cols());
        for (int i=0; i<cur_samples.rows(); ++i)
            for (int j=0; j<cur_samples.cols(); ++j)
                cur_samples(i, j) = (*samples)(mask_ids[i], j);
    }

// #if CV_MAJOR_VERSION>=2 && CV_MINOR_VERSION>=4

//     cv::EM em_model(num_labels, cv::EM::COV_MAT_DIAGONAL, term_crit);
//     em_model.train(cur_samples, cv::noArray(), cvlabels, cvprobs);

// #else

//     CvEM em_model;
//     CvEMParams em_params;
//     em_params.term_crit = term_crit;
//     em_params.nclusters = num_labels;
//     em_params.cov_mat_type = CvEM::COV_MAT_DIAGONAL;
//     em_model.train(cur_samples, cv::Mat(), em_params, &cvlabels);
//     cvprobs = em_model.getProbs();

// #endif

    labels.resize(mask_ids.size());
    probs.resize(mask_ids.size());
    for (unsigned int i=0; i<labels.size(); ++i)
    {
        labels[i] = cvlabels(i, 0);
        probs[i].resize(num_labels);
        for (int k=0; k<num_labels; ++k)
        {
            probs[i][k] = cvprobs(i, k);
            //std::cout <<face_probs[i][k] << " ";
        }
        //std::cout << "\n";
    }
}*/
