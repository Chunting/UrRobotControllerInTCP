//
// Created by longhuicai on 17-4-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EXPONENTIALFILTER_H
#define PROJECT_EXPONENTIALFILTER_H

#include <vector>

class ExponentialFilter {
public:
	ExponentialFilter(){}
	ExponentialFilter(int size, float factor);
	virtual ~ExponentialFilter();

	void init(int size, float factor);

	void setFactor(float factor) { m_factor = factor; }
	float getFactor() { return m_factor; }

	bool doFilter(std::vector<double>& source, std::vector<double>& result );

	int getSize() { return m_size; }

protected:
	bool m_binit;
	int  m_size;
	float m_factor;
	std::vector<double> m_value;
};


#endif //PROJECT_EXPONENTIALFILTER_H
