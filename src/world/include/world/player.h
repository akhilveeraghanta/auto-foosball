#pragma once

/*
 * A player is one of the humans/robot atatched to the foosball stick
 *
 */
class Player
{
    public:
        Player();
        virtual ~Player();

        int getPosition();
        int getPlayerID();

        void setPosition(int position);
        void setPlayerID();

    private:
        int position_cm;
        int player_id;
};
