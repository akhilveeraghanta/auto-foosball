#pragma once
#include <player.h>

class Stick
{
    public:
        Stick();
        virtual ~Stick();

        void setAngle(int angle);
        int getAngle();

    private:
        Player left;
        Player centre;
        Player right;
};

#endif /* STICK_H */
