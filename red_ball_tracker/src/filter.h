#include <vector>


namespace rbt {

class filter
{
  std::vector<float> _values;
  std::vector<float> _coeffs;
  unsigned _head;
  unsigned const _size;

public:

  filter(std::vector<float> const& coeffs);
  ~filter(void);

  float operator ()(float value);
};

} // rbt
