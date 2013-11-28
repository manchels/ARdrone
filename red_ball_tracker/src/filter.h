#include <vector>


namespace rbt {

class filter
{
  std::vector<float> _values;
  std::vector<float> _coeffs;
  unsigned _head;
  unsigned const _size;

public:

  filter(std::vector<float> const& coeffs, float start = 0.f);
  ~filter(void);

  float reset(float start = 0.f);
  float operator ()(float value);
};

} // rbt
