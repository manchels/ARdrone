#include "filter.h"


namespace rbt {


filter::filter(const std::vector<float> &coeffs, float start)
  : _values(coeffs.size(), 0)
  , _coeffs(coeffs)
  , _head(0)
  , _size(coeffs.size()) {}


filter::~filter(void) {}


float filter::reset(float start)
{ for (float& val : _values) val = start; return start; }


float filter::operator ()(float value)
{
  // Set value.
  {
    _values[(++_head) %= _size] = value;
  }

  // Compute filter.
  float res = 0.f;
  {
    unsigned j = 0;

    for (unsigned i = _head; i != 0; --i, ++j)
      res += _coeffs[j] * _values[i];
    for (unsigned i = _size - 1; i != _head; --i, ++j)
      res += _coeffs[j] * _values[i];
  }

  return res;
}


} // rbt
