#ifndef DTK_SIGN_H
#define DTK_SIGN_H

#include <ostream>
#include <string>

namespace dtk
{
    class dtkSign
    {
    public:
        const static dtkSign POSITIVE, NEGATIVE, ZERO;

    public:
        dtkSign(): mSign(0) {}
        dtkSign(const dtkSign &rhs): mSign(rhs.mSign) {}

        bool operator==(const dtkSign &rhs) const
        {
            return (mSign == rhs.mSign);
        }

        bool operator!=(const dtkSign &rhs) const
        {
            return !(operator==(rhs));
        }

        const dtkSign& operator=(const dtkSign &rhs)
        {
            mSign = rhs.mSign;
            return (*this);
        }

        const dtkSign& Opposite() const
        {
            if (mSign == 1) return dtkSign::NEGATIVE;
            if (mSign == -1) return dtkSign::POSITIVE;
            return dtkSign::ZERO;
        }

    private:
        dtkSign(int sign): mSign(sign)  {}
        char mSign;
    };

    inline std::ostream& operator<<(std::ostream &stream, const dtkSign &sign)
    {
        const static std::string strPOSITIVE("POSITIVE");
        const static std::string strNEGATIVE("NEGATIVE");
        const static std::string strZERO("ZERO");

        stream << (sign == dtkSign::ZERO? strZERO:
                    (sign == dtkSign::POSITIVE? strPOSITIVE: strNEGATIVE));
    }
};

#endif //DTK_SIGN_H
