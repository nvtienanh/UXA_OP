#include <trajectory_editor/frameview.h>

#include "ui_frameview.h"

#include <QLineEdit>
#include <QObject>
#include <QMessageBox>
#include <QFileDialog>

#include <ros/package.h>
#include <ros/console.h>

FrameView::FrameView(QWidget *parent)
{
	// Set up GUI
	ui = new Ui::FrameView;
	ui->setupUi(this);
	
	ui->allFrameBox->addItem(tr("Frame"));
	ui->allFrameBox->addItem(tr("All Frames"));
	
	ui->allBox->setChecked(true);
	setAllChecked(true);
	enableSupport(false);
	
	ui->leftBox->setChecked(true);
	ui->rightBox->setChecked(true);
	
	// Set up default support coeffs value
	ui->supportSlider->setValue(0.5 *100);
	ui->supportLeftSpin->setValue(0.5);
	ui->supportRightSpin->setValue(0.5);
	
	// Set up eventFilter
	ui->supportSlider->installEventFilter(this);
	ui->supportLeftSpin->installEventFilter(this);
	ui->supportRightSpin->installEventFilter(this);
	
	ui->effortSpin->installEventFilter(this);
	ui->durationSlider->installEventFilter(this);
	ui->durationSpin->installEventFilter(this);
	
	ui->rollSpin->installEventFilter(this);
	ui->pitchSpin->installEventFilter(this);
	ui->yawSpin->installEventFilter(this);
	
	// Set up subscription for Roll/Pitch/Yaw values
	updateRPY = false;
	subscriber = nh.subscribe("/robotmodel/robot_state", 100, &FrameView::robotStateReceived, this);
	
	// Set up connections
	connect(ui->nameEdit, SIGNAL(textEdited(QString)), this, SLOT(nameChanged()));
	connect(ui->saveButton, SIGNAL(clicked(bool)), this, SLOT(saveFrame()));
	connect(ui->loadButton, SIGNAL(clicked(bool)), this, SLOT(loadFrame()));
	
	connect(ui->durationSlider, SIGNAL(valueChanged(int)), this, SLOT(durationSliderChanged()));
	connect(ui->durationSpin, SIGNAL(valueChanged(double)), this, SLOT(durationSpinChanged()));
	
	connect(ui->applyButton, SIGNAL(clicked(bool)), this, SLOT(changeEffort()));
	connect(ui->allBox, SIGNAL(toggled(bool)), this, SLOT(setAllChecked(bool)));
	
	// Support
	connect(ui->supportLeftSpin, SIGNAL(valueChanged(double)), this, SLOT(supportLeftSpinChanged()));
	connect(ui->supportRightSpin, SIGNAL(valueChanged(double)), this, SLOT(supportRightSpinChanged()));
	connect(ui->supportSlider, SIGNAL(valueChanged(int)), this, SLOT(supportSliderChanged()));
	
	connect(ui->supportEnableBox, SIGNAL(toggled(bool)), this, SLOT(enableSupport(bool)));
	
	// Roll Pitch Yaw
	connect(ui->rollSpin, SIGNAL(valueChanged(double)), this, SLOT(handleRollSpinChanged()));
	connect(ui->pitchSpin, SIGNAL(valueChanged(double)), this, SLOT(handlePitchSpinChanged()));
	connect(ui->yawSpin, SIGNAL(valueChanged(double)), this, SLOT(handleYawSpinChanged()));
	
	connect(ui->updateRollPitchYawButton, SIGNAL(clicked(bool)), this, SLOT(updateRollPitchYaw()));
}

void FrameView::robotStateReceived(const robotcontrol::RobotStateConstPtr& state)
{
	if(!frame || updateRPY == false)
		return;
	
	ui->rollSpin->setValue(state->FRoll);
	ui->pitchSpin->setValue(state->FPitch);
	ui->yawSpin->setValue(state->FYaw);
	
	updateRPY = false;
}

void FrameView::saveFrame()
{
	if(!frame)
		return;
	
	// Create motion with current frame
	motionfile::Motion frameMotion;
	
	frameMotion.motionName = frame->name;
	frameMotion.jointList = joints;
	frameMotion.playState = "init";
	frameMotion.preState =  "init";
	frameMotion.postState = "init";
	frameMotion.frames.push_back(frame);
	
	// Build path to save motion
	QString pathToSave;
	
	pathToSave.append(this->path);
	pathToSave.append(this->motionName + "_" + QString::fromStdString(frame->name) + ".frame");
	
	// Save
	frameMotion.save(pathToSave.toStdString());
	
	ROS_INFO("Saved frame as: %s" , pathToSave.toStdString().c_str());
	QMessageBox::information(0, "Info", "Successfully saved frame!\nPath:\n" + pathToSave);
}

void FrameView::loadFrame()
{
	if(!frame)
		return;
	
	// Get path of file to load
	QString dir = QString::fromStdString(ros::package::getPath("launch") + "/motions/");
    QString path = QFileDialog::getOpenFileName(this, tr("open file"), dir, QString("*.frame"));
	
	if(path.isEmpty())
		return;
	
	// Load frameMotion
	motionfile::Motion frameMotion;
	frameMotion.load(path.toStdString());
	
	if(frameMotion.frames.size() != 1) // Saved frame cant have more than 1 frame
		return;
	
	// Prepare frame
	KeyframePtr temp = frameMotion.frames[0];
	
	KeyframePtr loadedFrame(new motionfile::Keyframe);
	loadedFrame->name = frameMotion.motionName;
	loadedFrame->id = frame->id;
	loadedFrame->duration = temp->duration;
	loadedFrame->joints = temp->joints;
	loadedFrame->support = temp->support;
	
	// Reorganize joints in frame to match current jointList of opened motion
	int index = 0;
	for(unsigned i = 0; i < joints.size(); i++) // For each joint in frame
	{
		for(unsigned j = 0; j < frameMotion.jointList.size(); j++) // Find position in loaded motion
		{
			if(frameMotion.jointList.at(j) == joints.at(i))
			{
				index = j;
				break;
			}
			index = i;
		}
		
		loadedFrame->joints[i] = temp->joints[index]; // Reorganize
	}
	
	frameLoaded(loadedFrame);
	ROS_INFO("Loaded frame: %s",loadedFrame->name.c_str());
}

void FrameView::setPathAndName(QString path)
{
	QStringList pieces = path.split("/");
	
	QString pathNoExtension;
	
	for(int i = 0; i < pieces.size()-1; i++)
		pathNoExtension.append(pieces.at(i) + "/");
	this->path = pathNoExtension;
	
	QStringList pieces2 = pieces.last().split(".");
	this->motionName = pieces2.first();
}

QString FrameView::getPathToDir()
{
	return path;
}

void FrameView::changeEffort()
{
	if(!frame)
		return;
	
	std::vector<int> jointsToApply;
	this->findIndices(jointsToApply);
	
	if(ui->allFrameBox->currentIndex() == 0) // Apply to current frame
	{
		ROS_INFO("Apply effort to one frame");
		
		for(unsigned i = 0; i < jointsToApply.size(); i++)
		{
			int id = jointsToApply.at(i);
			if(id < 0)
				continue;

			frame->joints[id].effort = ui->effortSpin->value();
		}
	}
	else // Apply to all frames
	{
		ROS_INFO("Apply effort to all frames");
		changeEffortForAllFrames(jointsToApply, this->getEffort());
	}

	updateFrame();
}

void FrameView::updateJointList(const std::vector<std::string> &jointList)
{
	this->joints = jointList;
}

double FrameView::getEffort()
{
	return ui->effortSpin->value();
}

void FrameView::nameChanged()
{
	if(!frame)
		return;
	
	QString name = ui->nameEdit->text();
	
	if(name.isEmpty())
	{
		ui->nameEdit->setStyleSheet("QLineEdit { background: rgb(235, 150, 150); }");
		ui->saveButton->setEnabled(false);
	}
	else
	{
		ui->nameEdit->setStyleSheet("QLineEdit { background: rgb(255, 255, 255); }");
		ui->saveButton->setEnabled(true);
		
		frame->name = name.toStdString();
		updateFrame();
	}
}

void FrameView::setFrame(KeyframePtr frame)
{
	this->frame = frame;
	
	if(!frame)
		return;
	
	ui->saveButton->setEnabled(true);
	
	// Set name of frame
	ui->nameEdit->setText(QString::fromStdString(frame->name));
	ui->nameEdit->setStyleSheet("QLineEdit { background: rgb(255, 255, 255); }");
	
	// Set duration values
	ui->durationSlider->blockSignals(true);
	ui->durationSlider->setValue(frame->duration * 100);
	ui->durationSlider->blockSignals(false);
	
	ui->durationSpin->blockSignals(true);
	ui->durationSpin->setValue(frame->duration);
	ui->durationSpin->blockSignals(false);
	
	setSupportFromFrame();
	
	// Set roll/pitch/yaw values
	ui->rollSpin->setValue(frame->roll);
	ui->pitchSpin->setValue(frame->pitch);
	ui->yawSpin->setValue(frame->yaw);
}

void FrameView::enableSupport(bool enable)
{
	if(!frame)
		return;
	
	if(!enable)
		frame->support = "";
	else
		setCurrentSupport();
	
	ui->supportEnableBox->blockSignals(true);
	ui->supportEnableBox->setChecked(enable);
	ui->supportEnableBox->blockSignals(false);
	
	ui->supportLeftSpin->setEnabled(enable);
	ui->supportRightSpin->setEnabled(enable);
	ui->supportSlider->setEnabled(enable);
}

void FrameView::setCurrentSupport()
{
	double left = ui->supportLeftSpin->value();
	double right = ui->supportRightSpin->value();
	
	std::ostringstream ss;
	ss << left;
	ss << " ";
	ss << right;
	
	frame->support = ss.str();
}

void FrameView::setSupportFromFrame()
{
	if (!frame)
		return;
	
	if(frame->support == "")
	{
		enableSupport(false);
		
		// Set up default support coeffs value
		ui->supportSlider->blockSignals(true);
		ui->supportLeftSpin->blockSignals(true);
		ui->supportRightSpin->blockSignals(true);
		
		ui->supportSlider->setValue(0.5 *100);
		ui->supportLeftSpin->setValue(0.5);
		ui->supportRightSpin->setValue(0.5);
		
		ui->supportSlider->blockSignals(false);
		ui->supportLeftSpin->blockSignals(false);
		ui->supportRightSpin->blockSignals(false);
		
		return;
	}
	
	// Parse support
	double left;
	double right;
	
	if(parseSupport(QString::fromStdString(frame->support), left, right) == false)
	{
		enableSupport(false);
		return;
	}
	
	ui->supportSlider->blockSignals(true);
	ui->supportLeftSpin->blockSignals(true);
	ui->supportRightSpin->blockSignals(true);
	
	ui->supportLeftSpin->setValue(left);
	ui->supportRightSpin->setValue(right);
	ui->supportSlider->setValue(right * 100);
	
	ui->supportSlider->blockSignals(false);
	ui->supportLeftSpin->blockSignals(false);
	ui->supportRightSpin->blockSignals(false);
	
	enableSupport(true);
}

bool FrameView::parseSupport(QString support, double &left, double &right)
{
	QStringList pieces = support.split(" ");
	
	if(pieces.size() !=2)
	{
		ROS_ERROR("Error when parsing support coefficients!");
		return false;
	}
	
	left = pieces.at(0).toDouble();
	right = pieces.at(1).toDouble();
	
	return true;
}

void FrameView::durationSpinChanged()
{
	if (!frame)
		return;
	
	ui->durationSlider->blockSignals(true);
	ui->durationSlider->setValue(ui->durationSpin->value() * 100);
	ui->durationSlider->blockSignals(false);
	
	frame->duration = ui->durationSpin->value();
}

void FrameView::durationSliderChanged()
{
	if (!frame)
		return;
	
	ui->durationSpin->blockSignals(true);
	ui->durationSpin->setValue(ui->durationSlider->value() / 100.0);
	ui->durationSpin->blockSignals(false);
	
	frame->duration = ui->durationSpin->value();
}

void FrameView::supportLeftSpinChanged()
{
	if (!frame)
		return;
	
	ui->supportSlider->blockSignals(true);
	ui->supportSlider->setValue((1 - ui->supportLeftSpin->value()) * 100);
	ui->supportSlider->blockSignals(false);
	
	ui->supportRightSpin->blockSignals(true);
	ui->supportRightSpin->setValue(1 - ui->supportLeftSpin->value());
	ui->supportRightSpin->blockSignals(false);
	
	setCurrentSupport();
}

void FrameView::supportRightSpinChanged()
{
	if (!frame)
		return;
	
	ui->supportSlider->blockSignals(true);
	ui->supportSlider->setValue(ui->supportRightSpin->value() * 100);
	ui->supportSlider->blockSignals(false);
	
	ui->supportLeftSpin->blockSignals(true);
	ui->supportLeftSpin->setValue(1 - ui->supportRightSpin->value());
	ui->supportLeftSpin->blockSignals(false);
	
	setCurrentSupport();
}

void FrameView::supportSliderChanged()
{
	if (!frame)
		return;
	
	ui->supportLeftSpin->blockSignals(true);
	ui->supportLeftSpin->setValue(1 - ui->supportSlider->value() / 100.0);
	ui->supportLeftSpin->blockSignals(false);
	
	ui->supportRightSpin->blockSignals(true);
	ui->supportRightSpin->setValue(ui->supportSlider->value()/100.0);
	ui->supportRightSpin->blockSignals(false);
	
	setCurrentSupport();
}

void FrameView::handleRollSpinChanged()
{
	if(!frame)
		return;
	
	frame->roll = ui->rollSpin->value();
}

void FrameView::handlePitchSpinChanged()
{
	if(!frame)
		return;
	
	frame->pitch = ui->pitchSpin->value();
}

void FrameView::handleYawSpinChanged()
{
	if(!frame)
		return;
	
	frame->yaw = ui->yawSpin->value();
}

void FrameView::updateRollPitchYaw()
{
	if(!frame)
		return;
	
	updateRPY = true;
}

int FrameView::nameToIndex(std::string name)
{
	for (unsigned i = 0; i < joints.size(); i++)
	{
		if (joints[i] == name)
			return i;
	}
	return -1;
}

void FrameView::findIndices(std::vector<int> &indices)
{
	if(ui->headBox->isChecked()) // Head
	{
		indices.push_back(nameToIndex("head_pitch"));
		indices.push_back(nameToIndex("neck_yaw"));
	}
	
	if(ui->armBox->isChecked()) // Arm
	{
		if(ui->leftBox->isChecked()) // left
		{
			indices.push_back(nameToIndex("left_shoulder_pitch"));
			indices.push_back(nameToIndex("left_shoulder_roll"));
			indices.push_back(nameToIndex("left_elbow_pitch"));
		}
		if(ui->rightBox->isChecked()) // right
		{
			indices.push_back(nameToIndex("right_shoulder_pitch"));
			indices.push_back(nameToIndex("right_shoulder_roll"));
			indices.push_back(nameToIndex("right_elbow_pitch"));
		}
	}
	
	if(ui->legPitchBox->isChecked()) // Leg pitch
	{
		if(ui->leftBox->isChecked()) // left
		{
			indices.push_back(nameToIndex("left_hip_pitch"));
			indices.push_back(nameToIndex("left_knee_pitch"));
			indices.push_back(nameToIndex("left_ankle_pitch"));
		}
		if(ui->rightBox->isChecked()) // right
		{
			indices.push_back(nameToIndex("right_hip_pitch"));
			indices.push_back(nameToIndex("right_knee_pitch"));
			indices.push_back(nameToIndex("right_ankle_pitch"));
		}
	}
	
	if(ui->legRollBox->isChecked()) // Leg roll
	{
		if(ui->leftBox->isChecked()) // left
		{
			indices.push_back(nameToIndex("left_hip_roll"));
			indices.push_back(nameToIndex("left_ankle_roll"));
		}
		if(ui->rightBox->isChecked()) // right
		{
			indices.push_back(nameToIndex("right_hip_roll"));
			indices.push_back(nameToIndex("right_ankle_roll"));
		}
	}
	
	if(ui->legYawBox->isChecked()) // Leg yaw
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_hip_yaw"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_hip_yaw"));
	}
	
	if(ui->anklePitchBox->isChecked()) // Ankle pitch
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_ankle_pitch"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_ankle_pitch"));
	}
	
	if(ui->ankleRollBox->isChecked()) // Ankle roll
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_ankle_roll"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_ankle_roll"));
	}
	
	if(ui->hipPitchBox->isChecked()) // Hip pitch
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_hip_pitch"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_hip_pitch"));
	}
	
	if(ui->hipRollBox->isChecked()) // Hip roll
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_hip_roll"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_hip_roll"));
	}
	
	if(ui->hipYawBox->isChecked()) // Hip yaw
	{
		if(ui->leftBox->isChecked()) // left
			indices.push_back(nameToIndex("left_hip_yaw"));
		
		if(ui->rightBox->isChecked()) // right
			indices.push_back(nameToIndex("right_hip_yaw"));
	}
}

bool FrameView::eventFilter(QObject *object, QEvent *event)
{
    if(event->type() == QEvent::Wheel) // Ignore wheel scrolling
	{
		if(object == ui->supportSlider)
			return true;
		else if (object == ui->supportLeftSpin || object == ui->supportRightSpin)
			return true;
		else if (object == ui->effortSpin || object == ui->durationSlider || object == ui->durationSpin)
			return true;
		else if(object == ui->pitchSpin || object == ui->rollSpin || object == ui->yawSpin)
			return true;
    }
    
    return false;
}

void FrameView::setAllChecked(bool flag)
{
	ui->headBox->setChecked(flag);
	ui->armBox->setChecked(flag);
	ui->legPitchBox->setChecked(flag);
	ui->legRollBox->setChecked(flag);
	ui->legYawBox->setChecked(flag);
	ui->anklePitchBox->setChecked(flag);
	ui->ankleRollBox->setChecked(flag);
	ui->hipRollBox->setChecked(flag);
	ui->hipPitchBox->setChecked(flag);
	ui->hipYawBox->setChecked(flag);
}

FrameView::~FrameView()
{
	delete ui;
}
