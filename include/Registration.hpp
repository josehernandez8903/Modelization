/*
 * Registration.hpp
 *
 *  Created on: Dec 6, 2013
 *      Author: jose
 */

#ifndef REGISTRATION_HPP_
#define REGISTRATION_HPP_

#include "planeDetection.hpp"


namespace Modelization {

class Registration {
public:
	Registration();
	virtual ~Registration();

	void run(const pcl::PCLPointCloud2ConstPtr &cloud_blob
			, PCLXYZRGBPoint::Ptr cloud_filtered
			, PCLXYZRGBPoint::Ptr cloud_r);
};

} /* namespace Modelization */
#endif /* REGISTRATION_HPP_ */
