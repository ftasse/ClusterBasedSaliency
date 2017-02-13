#ifndef SEGMENTATION_SEGMENT_H
#define SEGMENTATION_SEGMENT_H

#include <vector>
#include <Eigen/Core>

struct Segment
{
	std::vector<int> assignments;
	Eigen::VectorXf mean;

	Segment() {}

	size_t num_points() const 
	{
		return assignments.size();
	}

	size_t indexAt(int pos) const 
	{
		// assert(pos >= 0 && pos < assignments.size());
		return assignments[pos];
	}
};

#endif // SEGMENTATION_SEGMENT_H
