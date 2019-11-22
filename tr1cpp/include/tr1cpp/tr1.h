#ifndef TR1CPP__TR1_H
#define TR1CPP__TR1_H

#include <sstream>
#include <tr1cpp/segment.h>

namespace tr1cpp
{
	class TR1
	{
		private:
		public:
			TR1();
			~TR1();

            Segment<2> armRight;
            Joint getJoint(std::string jointName);
			void setJoint(tr1cpp::Joint joint);
	};
}

#endif
