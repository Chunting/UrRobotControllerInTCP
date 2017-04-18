//
// Created by longhuicai on 17-4-13.
//

#include "ExponentialFilter.h"

ExponentialFilter::ExponentialFilter(int size, float factor)
{
	init(size, factor);
}

ExponentialFilter::~ExponentialFilter() {
}

void ExponentialFilter::init(int size, float factor) {
	m_binit = true;
	m_size = size;
	m_factor = factor;
	m_value.resize(size);
}

bool ExponentialFilter::doFilter(std::vector<double>& source, std::vector<double>& result) {
	bool ret = false;
	if (m_size == source.size() )
	{
		result.resize(m_size);
		if (m_binit)
		{
			m_value = source;
			result = m_value;
			m_binit = false;
		}
		else
		{
			for (size_t i = 0; i < m_size; i++)
			{
				m_value.at(i) += m_factor*(source.at(i) - m_value.at(i) );
//				m_value.at(i) = (1 - m_factor)*m_value.at(i) + m_factor*source.at(i);
			}
			result = m_value;
		}
	}
	return ret;
}
