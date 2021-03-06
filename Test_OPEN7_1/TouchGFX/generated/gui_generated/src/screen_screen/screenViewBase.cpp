/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen_screen/screenViewBase.hpp>
#include <touchgfx/Color.hpp>
#include "BitmapDatabase.hpp"

screenViewBase::screenViewBase() :
    interaction2Counter(0),
    buttonCallback(this, &screenViewBase::buttonCallbackHandler)
{

    __background.setPosition(0, 0, 1024, 600);
    __background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    box1.setPosition(0, 0, 1024, 600);
    box1.setColor(touchgfx::Color::getColorFrom24BitRGB(42, 89, 14));

    button1.setXY(381, 278);
    button1.setBitmaps(touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_MEDIUM_ID), touchgfx::Bitmap(BITMAP_BLUE_BUTTONS_ROUND_MEDIUM_PRESSED_ID));
    button1.setAction(buttonCallback);

    add(__background);
    add(box1);
    add(button1);
}

void screenViewBase::setupScreen()
{

}

//Handles delays
void screenViewBase::handleTickEvent()
{
    if(interaction2Counter > 0)
    {
        interaction2Counter--;
        if(interaction2Counter == 0)
        {
            //Interaction3
            //When Interaction2 completed change screen to Screen1
            //Go to Screen1 with screen transition towards North
            application().gotoScreen1ScreenSlideTransitionNorth();
        }
    }
}

//Called when the screen is done with transition/load
void screenViewBase::afterTransition()
{
    //Interaction2
    //When screen is entered delay
    //Delay for 5000 ms (300 Ticks)
    interaction2Counter = INTERACTION2_DURATION;
}

void screenViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &button1)
    {
        //Interaction1
        //When button1 clicked change screen to Screen1
        //Go to Screen1 with screen transition towards East
        application().gotoScreen1ScreenCoverTransitionEast();
    }
}
