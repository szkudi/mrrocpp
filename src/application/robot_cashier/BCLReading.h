/*
 * BCLReading.h
 *
 *  Created on: 18-10-2011
 *      Author: szkudi
 */

#ifndef BCLREADING_H_
#define BCLREADING_H_

#include "application/visual_servoing/Reading.h"
#include "application/visual_servoing/ImagePosition.h"

#include <boost/numeric/ublas/vector.hpp>

namespace Types {
namespace Mrrocpp_Proxy {

class Triple{
public :
	Triple(){x = 0; y = 0; z = 0;};
	Triple(double _x, double _y, double _z){x = _x; y = _y; z = _z;};

private:
	friend class boost::serialization::access;

	double x;
	double y;
	double z;

	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & x;
		ar & y;
		ar & z;
	}
};

/**
 *
 */
class BCLReading: public Reading
{
public:
	BCLReading() : code_found(false)
	{
	}

	virtual ~BCLReading()
	{
	}

	virtual BCLReading* clone()
	{
		return new BCLReading(*this);
	}

	virtual void send(boost::shared_ptr<xdr_oarchive<> > & ar){
		*ar<<*this;
	}

	int getCount(){
		return this->regions.size();
	}

//	Triple getElementAt(int i){
//		return this->regions[i];
//	}

	Types::ImagePosition& getElementAt(int i){
		return this->regions[i];
	}

	void addElement(Types::ImagePosition elem){
		code_found = true;
//		regions.push_back(elem);
		regions.resize(regions.size() + 1);
		regions.insert_element(regions.size() - 1, elem);	}

	bool codeFound(){
		return code_found;
	}

private:
	bool code_found;

//	std::vector<Triple> regions;
//	std::vector<Types::ImagePosition> regions;
	boost::numeric::ublas::vector<Types::ImagePosition> regions;

	friend class boost::serialization::access;
	template <class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
//		std::cout << "PBReading::serialize()\n";
		ar & boost::serialization::base_object <Reading>(*this);

		ar & code_found;
		ar & regions;
//		for(int i = 0; i < regions.size(); ++i){
//			ar & regions[i];
//			std::cout << regions[i].elements[0];
//		}
	}
//	template <class Archive>
//	void save (Archive & ar, const unsigned int version) const
//	{
////		std::cout << "PBReading::serialize()\n";
//		ar & boost::serialization::base_object <Reading>(*this);
//
//		ar & code_found;
//		ar & regions.size();
//		for(int i = 0; i < regions.size(); ++i){
//			ar & regions[i];
//			std::cout << regions[i].elements[0];
//		}
//	}
//	template <class Archive>
//	void load(Archive & ar, const unsigned int version)
//	{
//		int size;
//		Types::ImagePosition tmp;
//
////		std::cout << "PBReading::serialize()\n";
//		ar & boost::serialization::base_object <Reading>(*this);
//
//		ar & code_found;
//		ar & size;
//		for(int i = 0; i < size; ++i){
//			ar & tmp;
//			regions.push_back(tmp);
//			std::cout << tmp.elements[0];
//		}
//	}
};

}//namespace Mrrocpp_Proxy
}//namespace Types




#endif /* BCLREADING_H_ */
