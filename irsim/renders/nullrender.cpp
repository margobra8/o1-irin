#include "nullrender.h"

/******************************************************************************/
/******************************************************************************/

CNullRender::CNullRender(CSimulator* pc_simulator) : CRender(pc_simulator)
{
};

/******************************************************************************/
/******************************************************************************/

void CNullRender::DrawEpuck(CEpuck* pc_epuck, float f_color_red, float f_color_green, float f_color_blue)
{
}

/******************************************************************************/
/******************************************************************************/

void CNullRender::DrawPuck(CPuck* pc_puck, float f_color_red, float f_color_green, float f_color_blue)
{
}

/******************************************************************************/
/******************************************************************************/

void CNullRender::DrawArena(CArena* pc_arena)
{ 
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawColoredWall(CColoredWall* pc_w)
{
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawLight(CLightObject* pc_l)
{
}

/******************************************************************************/
/******************************************************************************/

void CNullRender::Start()
{
    while (!m_pcSimulator->HasEnded()) 
    {
        m_pcSimulator->TakeSimulationStep();        
    }
}

/******************************************************************************/
/******************************************************************************/

void CNullRender::Draw2DLine(dVector2 v_orig, dVector2 v_dest, float height, float color_red, float color_green, float color_blue){	
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawSbot(CSbot* pc_sbot, float f_color_red, float f_color_green, float f_color_blue)
{
    // Do absolutely nothing
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawStoy(CStoy* pc_stoy, float f_color_red, float f_color_green, float f_color_blue)
{
    // Nothing at all
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawPalet(CPalet* pc_palet, float f_color_red, float f_color_green, float f_color_blue)
{
    // Still nothing 
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawLight(CLightEmitter* pc_light)
{
    // boring class, does really nothing
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawSound(CSoundEmitter* pc_sound)
{
    // boooooring (*_*)
}

/******************************************************************************/
/******************************************************************************/
/*
void CNullRender::DrawIntensitySound(CSoundEmitter* pc_sound, double f_intensity)
{

}

/******************************************************************************/
/******************************************************************************/