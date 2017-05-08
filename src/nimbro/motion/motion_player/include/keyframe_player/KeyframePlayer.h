#ifndef KEYFRAMEPLAYER_H_
#define KEYFRAMEPLAYER_H_
#include "keyframe_player/Keyframe.h"
#include <QList>

namespace kf_player
{

class KeyframePlayer
{
public:

	double V;
	double A;
	double VX;

	QList<Keyframe> keyframes;
	QList<Keyframe> commands;
	Keyframe currentState;

public:
	KeyframePlayer();
	virtual ~KeyframePlayer() {};

	void setA(double A);
	void setV(double V);
	void setVX(double VX);

	void reset();
	void clear();
	
	bool addKeyframe(Keyframe kf);
	bool addKeyframe(double t, double x, double v, double effort, double suppLeftLeg, double suppRightLeg,double pGain, double iGain, double dGain, double limit, gainSelectEnum gainSelect, double roll, double pitch, double yaw) { return addKeyframe(Keyframe(t, x, v, effort, suppLeftLeg, suppRightLeg, pGain,iGain,dGain,limit,gainSelect, roll, pitch,yaw)); }

	void calculateCommands();

	bool atEnd();
	Keyframe step(double t);
	Keyframe getCurrentState();
	Keyframe evaluateAt(double t);
	double totalTime();
	double currentTime();
	double minimumTime();

private:
	void transformState(double a, double t, Keyframe& kf);
};

}

#endif // KEYFRAMEPLAYER_H_
