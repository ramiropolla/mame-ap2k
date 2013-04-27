#ifndef __DEBUG_QT_MAIN_WINDOW_H__
#define __DEBUG_QT_MAIN_WINDOW_H__

#include <QtGui/QtGui>
#include <vector>

#include "debug/dvdisasm.h"

#include "debugqtview.h"
#include "debugqtwindow.h"

class DasmDockWidget;
class ProcessorDockWidget;


//============================================================
//  The Main Window.  Contains processor and dasm docks.
//============================================================
class MainWindow : public WindowQt
{
	Q_OBJECT

public:
	MainWindow(running_machine* machine, QWidget* parent=NULL);
	virtual ~MainWindow() {}

	void setProcessor(device_t* processor);


protected:
	// Used to intercept the user clicking 'X' in the upper corner
	void closeEvent(QCloseEvent* event);

	// Used to intercept the user hitting the up arrow in the input widget
	bool eventFilter(QObject* obj, QEvent* event);


private slots:
	void toggleBreakpointAtCursor(bool changedTo);
	void runToCursor(bool changedTo);
	void rightBarChanged(QAction* changedTo);

	void executeCommand(bool withClear=true);

	// Closing the main window actually exits the program
	void debugActClose();


private:
	// Widgets and docks
	QLineEdit* m_inputEdit;
	DebuggerView* m_consoleView;
	ProcessorDockWidget* m_procFrame;
	DasmDockWidget* m_dasmFrame;

	// Terminal history
	int m_historyIndex;
	std::vector<QString> m_inputHistory;
	void addToHistory(const QString& command);
};


//============================================================
//  Docks with the Main Window.  Disassembly.
//============================================================
class DasmDockWidget : public QWidget
{
	Q_OBJECT

public:
	DasmDockWidget(running_machine* machine, QWidget* parent=NULL) :
		QWidget(parent),
		m_machine(machine)
	{
		m_dasmView = new DebuggerView(DVT_DISASSEMBLY,
										m_machine,
										this);

		// Force a recompute of the disassembly region
		downcast<debug_view_disasm*>(m_dasmView->view())->set_expression("curpc");

		QVBoxLayout* dvLayout = new QVBoxLayout(this);
		dvLayout->addWidget(m_dasmView);
		dvLayout->setContentsMargins(4,0,4,0);
	}


	virtual ~DasmDockWidget() {}


	DebuggerView* view() { return m_dasmView; }


	QSize minimumSizeHint() const
	{
		return QSize(150,150);
	}


	QSize sizeHint() const
	{
		return QSize(150,200);
	}


private:
	DebuggerView* m_dasmView;

	running_machine* m_machine;
};


//============================================================
//  Docks with the Main Window.  Processor information.
//============================================================
class ProcessorDockWidget : public QWidget
{
	Q_OBJECT

public:
	ProcessorDockWidget(running_machine* machine,
						QWidget* parent=NULL) :
		QWidget(parent),
		m_processorView(NULL),
		m_machine(machine)
	{
		m_processorView = new DebuggerView(DVT_STATE,
											m_machine,
											this);
		m_processorView->setFocusPolicy(Qt::NoFocus);

		QVBoxLayout* cvLayout = new QVBoxLayout(this);
		cvLayout->addWidget(m_processorView);
		cvLayout->setContentsMargins(4,0,4,2);
	}


	virtual ~ProcessorDockWidget() {}


	DebuggerView* view() { return m_processorView; }


	QSize minimumSizeHint() const
	{
		return QSize(150,300);
	}


	QSize sizeHint() const
	{
		return QSize(200,300);
	}


private:
	DebuggerView* m_processorView;

	running_machine* m_machine;
};


//=========================================================================
//  A way to store the configuration of a window long enough to read/write.
//=========================================================================
class MainWindowQtConfig : public WindowQtConfig
{
public:
	MainWindowQtConfig() :
		WindowQtConfig(WIN_TYPE_MAIN),
		m_rightBar(0),
		m_windowState()
	{}
	
	~MainWindowQtConfig() {}
	
	// Settings
	int m_rightBar;
	QByteArray m_windowState;
	
	void buildFromQWidget(QWidget* widget);
	void applyToQWidget(QWidget* widget);
	void addToXmlDataNode(xml_data_node* node) const;
	void recoverFromXmlNode(xml_data_node* node);
};



#endif
