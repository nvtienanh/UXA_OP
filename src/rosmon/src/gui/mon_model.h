// Qt model for a rosmon instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MON_MODEL_H
#define MON_MODEL_H

#include <QAbstractTableModel>

#include <rosmon/State.h>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

namespace rosmon
{

class MonModel : public QAbstractTableModel
{
Q_OBJECT
public:
	explicit MonModel(ros::NodeHandle& nh, QObject* parent = 0);
	virtual ~MonModel();

	virtual int rowCount(const QModelIndex & parent) const override;
	virtual int columnCount(const QModelIndex & parent) const override;
	virtual QVariant data(const QModelIndex & index, int role) const override;

	virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

	inline QString namespaceString() const
	{ return m_namespace; }

	QString nodeName(int row) const;
public Q_SLOTS:
	void setNamespace(const QString& ns);
	void unsubscribe();
Q_SIGNALS:
	void stateReceived(const rosmon::StateConstPtr& state);
private Q_SLOTS:
	void updateState(const rosmon::StateConstPtr& state);
private:
	struct Entry
	{
		inline bool operator<(const Entry& other) const
		{ return name < other.name; }

		QString name;
		int state;
		int restartCount;
	};

	enum Column
	{
		COL_NAME,
		COL_RESTART_COUNT,

		COL_COUNT
	};

	ros::NodeHandle m_nh;

	QString m_namespace;

	std::vector<Entry> m_entries;

	ros::Subscriber m_sub_state;
};

}

#endif
