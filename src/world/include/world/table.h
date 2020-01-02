#pragma once

class Table
{
    public:
        Table();
        virtual ~Table();
    private:
        Stick front;
        Stick back; // TODO better naming
};

#endif /* TABLE_H */
