#ifndef DTK_TIME_H
#define DTK_TIME_H

#include <ctime>

namespace dtk
{
    //! Use it to track time
    class dtkTime
    {
        friend dtkTime operator-(const dtkTime &lhs, const dtkTime &rhs);

    public:
        dtkTime(const dtkTime &time)
            :mClicks(time.mClicks)
        {}

        explicit dtkTime(clock_t time = clock())
            :mClicks(time)
        {}

        inline void SetToCurrentTime()
        {
            mClicks = clock();
        }

        inline void SetTime(clock_t time)
        {
            mClicks = time;
        }

        inline clock_t GetTime() const
        {
            return mClicks;
        }

        inline float Seconds() const
        {
            return ((float)mClicks) / CLOCKS_PER_SEC;
        }

        inline bool operator==(const dtkTime &rhs)
        {
            return mClicks == rhs.mClicks;
        }

        inline void Advance(double adv)
        {
            mClicks += adv * (double)CLOCKS_PER_SEC;
        }

    private:
        clock_t mClicks;
    };

    inline dtkTime operator-(const dtkTime &lhs, const dtkTime &rhs)
    {
        return dtkTime(lhs.GetTime() - rhs.GetTime());
    }
}

#endif //DTK_TIME
