#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QImage>
#include <QShortcut>
#include <QDir>
#include <QFile>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
   QMainWindow(parent),
   ui(new Ui::MainWindow)
{
   ui->setupUi(this);

   connect(this->ui->m_addLabelButton, SIGNAL(clicked()),
           this, SLOT(addLabelButtonSlot()));

   connect(this->ui->m_nextImageButton, SIGNAL(clicked()),
           this, SLOT(showNextLabelSlot()));

   connect(this->ui->m_previousImagEButton, SIGNAL(clicked()),
           this, SLOT(showPreviousLabelSlot()));

   connect(this->ui->m_addLabelButton, SIGNAL(clicked()),
           this, SLOT(addNewLabel()));

   connect(this->ui->m_setLabelButton, SIGNAL(clicked()),
           this, SLOT(saveLabel()));

   m_currentImage = -1;

   QShortcut * shortcut = new QShortcut(QKeySequence(Qt::Key_N),this,SLOT(showNextLabelSlot()));
   QShortcut * shortcut2 = new QShortcut(QKeySequence(Qt::Key_P),this,SLOT(showPreviousLabelSlot()));
//   shortcut->setAutoRepeat(false);

}

MainWindow::~MainWindow()
{
   delete ui;
}

void MainWindow::addLabelButtonSlot()
{
      qDebug()<<"Add label button pressed.";
}


void MainWindow::parseInputPath(std::string path, std::string waypoint)
{

      vector<QString> allSweeps = listXmlInFolder(QString(path.c_str()), 10);
      std::sort(allSweeps.begin(), allSweeps.end());

      qDebug()<<"Found "<<allSweeps.size()<<" sweeps.";
      for (auto sweep : allSweeps)
      {
         qDebug()<<sweep;

         unsigned found = sweep.toStdString().find_last_of("/");
         std::string base_path = sweep.toStdString().substr(0,found+1);
         QStringList labelFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.jpg"));
         QStringList objectFiles = QDir(base_path.c_str()).entryList(QStringList("*object*.jpg"));

         for (auto label: labelFiles)
         {
            m_allMasks.push_back(QString(base_path.c_str()) + "/" + label);            
         }

         for (auto object: objectFiles)
         {
             m_allObjectFiles.push_back(QString(base_path.c_str()) + "/" + object);
         }

      }

      if (m_allMasks.size() != m_allObjectFiles.size())
      {
         qDebug()<<"Object and mask images are not equal !!!!! ";
      } else {
         qDebug()<<"Object and mask images are equal !!!!! ";
      }
}

vector<QString> MainWindow::listXmlInFolder(QString qrootFolder, int depth)
{
    if (depth > m_maxFolderDepth)
    {
        return std::vector<QString>();
    }


    std::vector<QString> toRet;

    QStringList childFolders = QDir(qrootFolder).entryList(QStringList("*"),
                                                    QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot,QDir::Time);

    QStringList filters;
    filters << "room.xml";

    for ( QString file : QDir(qrootFolder).entryList(filters, QDir::Files) )
    {
        toRet.push_back(qrootFolder+file);
    }

    for(QString childFolder : childFolders)
    {
        std::vector<QString> childXmls = listXmlInFolder(qrootFolder+childFolder+"/", depth+1);
        toRet.insert(toRet.end(),childXmls.begin(), childXmls.end());
    }

    return toRet;
}

void MainWindow::showNextLabelSlot()
{
   qDebug()<<"Showing the next label image"<<endl;
   m_currentImage++;

   if (m_currentImage >= m_allMasks.size())
   {
      m_currentImage = m_allMasks.size() - 1;
      return;
   }

   displayImage(m_currentImage, m_allMasks, m_allObjectFiles);

   // check that the assigned label matches the restriction imposed by the user
   QString currentLabel = this->ui->m_currentLabel->text();
   QString matchLabel = this->ui->m_matchLabel->text();
   qDebug()<<"Match label "<<matchLabel;

   if ((currentLabel != "") && (matchLabel != "") && (currentLabel != matchLabel)) {
        showNextLabelSlot();
   }

}

void MainWindow::showPreviousLabelSlot()
{
   qDebug()<<"Showing the previous label image"<<endl;
   m_currentImage--;

   if (m_currentImage <0) {
       m_currentImage = 0;
       return;
   }

   displayImage(m_currentImage, m_allMasks, m_allObjectFiles);

   // check that the assigned label matches the restriction imposed by the user
   QString currentLabel = this->ui->m_currentLabel->text();
   QString matchLabel = this->ui->m_matchLabel->text();

   if ((currentLabel != "") && (matchLabel != "") && (currentLabel != matchLabel)) {
        showPreviousLabelSlot();
   }
}

void MainWindow::displayImage(int image_number, std::vector<QString> allImageFiles, std::vector<QString> allObjectFiles)
{
   QImage image(allImageFiles[image_number]);
   QImage objectImage(allObjectFiles[image_number]);

   if (!image.isNull())
   {
      qDebug()<<"Now showing "<<allImageFiles[image_number];
   } else {
      qDebug()<<"Image "<<allImageFiles[image_number]<<"is not valid.";
   }

   this->ui->m_currentLabel->setText("");

   this->ui->m_rgb->setPixmap(QPixmap::fromImage(objectImage));
   this->ui->m_mask->setPixmap(QPixmap::fromImage(image));

   // check for predefined label
   QString label_file = allImageFiles[image_number];
   unsigned found = label_file.toStdString().find(".jpg");
   std::string base_path = label_file.toStdString().substr(0,found+1);
   QString new_file = QString(base_path.c_str()) + "txt";

   QFile textFile(new_file);
   if (!textFile.open(QIODevice::ReadOnly | QIODevice::Text))
         return;

   QTextStream textStream(&textFile);
   QString existing_label = textStream.readLine();
   this->ui->m_currentLabel->setText(existing_label);

   // display RGB image if available
   unsigned found2 = allImageFiles[image_number].toStdString().find("rgb_");
   std::string rgb_file = allImageFiles[image_number].toStdString().substr(0, found2+8);
   rgb_file+= ".jpg";
   qDebug()<<"Corresponding rgb file "<<rgb_file.c_str();
//   m_allRGBFiles.push_back(QString(base_path.c_str()) + "/" + rgb_file.c_str());
   QImage rgbImage(rgb_file.c_str());
   if (!rgbImage.isNull()){
       // display
        this->ui->m_rgbImage->setPixmap(QPixmap::fromImage(rgbImage));
   }

}

void MainWindow::readLabels(QString labels, QString display_labels)
{
   qDebug()<<"Parsing label files.";

   QStringList stringList;
   QFile textFile(labels);
   if (!textFile.open(QIODevice::ReadOnly | QIODevice::Text))
         return;

   m_labelFile = labels;
   QTextStream textStream(&textFile);
   while (true)
   {
       QString line = textStream.readLine();
       if (line.isNull())
           break;
       else
           stringList.append(line);
   }
   m_allLabels = stringList;

   stringList.clear();
   QFile textFile2(display_labels);
   if (!textFile2.open(QIODevice::ReadOnly | QIODevice::Text))
         return;

   m_displayLabelFile = display_labels;
   QTextStream textStream2(&textFile2);
   while (true)
   {
       QString line = textStream2.readLine();
       if (line.isNull())
           break;
       else
           stringList.append(line);
   }


   m_displayLabels = stringList;

   updateLabelLists(m_displayLabels);
}

void MainWindow::updateLabelLists(QStringList displayLabels)
{
   this->ui->m_label->clear();
   this->ui->m_label->addItems(displayLabels);
}

void MainWindow::addNewLabel()
{

   QString newLabel = this->ui->m_newLabel->text();
   QString newDisplayLabel = this->ui->m_newDisplayLabel->text();

   if ((newLabel == "") && (newDisplayLabel == ""))
   {
      qDebug()<<"Label empty"<<newLabel;
      return;
   }

   qDebug()<<"Adding a new label "<<newLabel;
   if (newDisplayLabel == "")
   {
      newDisplayLabel = newLabel;
   }

   m_allLabels.push_back(newLabel);
   m_displayLabels.push_back(newDisplayLabel);
   updateLabelLists(m_displayLabels);

   if ((m_labelFile == "") || (m_displayLabelFile == ""))
      return;

   QFile f1(m_labelFile);
   if (!f1.open(QIODevice::WriteOnly | QIODevice::Text))
         return;
   QTextStream out1(&f1);
   for (auto label : m_allLabels)
   {
      out1<<label<<"\n";
   }
   f1.close();

   QFile f2(m_displayLabelFile);
   if (!f2.open(QIODevice::WriteOnly | QIODevice::Text))
         return;
   QTextStream out2(&f2);
   for (auto label : m_displayLabels)
   {
      out2<<label<<"\n";
   }
   f2.close();

}

void MainWindow::saveLabel()
{
   QList<QListWidgetItem*> allselections = this->ui->m_label->selectedItems();
   if (allselections.size() == 0)
      return;

   QString display_label = this->ui->m_label->currentItem()->text();
   QString label = "";
   for (size_t i=0; i<m_displayLabels.size(); i++)
   {
      if (m_displayLabels[i] == display_label)
      {
         label = m_allLabels[i];
      }
   }

   if (m_currentImage == -1)
   {
      return;
   }

   QString label_file = m_allMasks[m_currentImage];
   unsigned found = label_file.toStdString().find(".jpg");
   std::string base_path = label_file.toStdString().substr(0,found+1);
   QString new_file = QString(base_path.c_str()) + "txt";

   qDebug()<<"Saving label(display) "<<display_label<<" actual "<<label<<"  at "<<new_file;

   QFile file(new_file);
   if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
         return;
   QTextStream out(&file);
   out<<label<<"\n";
   file.close();

   displayImage(m_currentImage, m_allMasks, m_allObjectFiles);
}
