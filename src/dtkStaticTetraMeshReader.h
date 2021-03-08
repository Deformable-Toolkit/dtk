#ifndef DTK_STATICTETRAMESHREADER_H
#define DTK_STATICTETRAMESHREADER_H

#include "dtkStaticTetraMesh.h"

#include <cstring>
#include <memory>
#include <boost/utility.hpp>

namespace dtk
{
	class dtkStaticTetraMeshReader: public boost::noncopyable
	{
	public:
		typedef std::shared_ptr<dtkStaticTetraMeshReader> Ptr;

		static dtkStaticTetraMeshReader::Ptr New()
		{
			return dtkStaticTetraMeshReader::Ptr(new dtkStaticTetraMeshReader());
		}

	public:
		virtual ~dtkStaticTetraMeshReader() {}

		void SetOutput(dtkStaticTetraMesh::Ptr mesh);
		void SetFileName(const char* filePath);

		bool Read();

	private:
        dtkStaticTetraMeshReader(); 

		dtkStaticTetraMesh::Ptr	mMesh;
		std::string			mFilePath;
	};
}

#endif //DTK_STATICTETRAMESHREADER_H

