//#ifndef SFTP_H
//#define SFTP_H

//#include <QtNetwork/QTcpServer>
//#include <QtNetwork/QTcpSocket>
//#include <libssh2.h>
//#include <libssh2_sftp.h>
//#include <QObject>
//#include <QFile>
//#include <protonet/sftpsession.h>

//class SFtp : public QObject
//{
//    Q_OBJECT
//public:
//    SFtp();
//    ~SFtp();
//    int getFile(const QString& remote_file,const QString& local_file,
//                 const QString& username ,const QString& password,const QString& host,qint16 port=22);
//    int putFile(const QString& local_file,const QString& remote_file,
//                const QString& username ,const QString& password,const QString& host,qint16 port=22);

////signals:
////    void dataTransferProgress(qint64,qint64);
////    void getProgressVal(int);

//public  Q_SLOTS:
//    void abort();
//    void upDateProgress(qint64,qint64);

//private:
//    void disconnect();
//    int connect(const QString& host, const QString& username ,const QString& password,int port=22);

//    int getFile(const QString& remotefile,const QString& localfile);
//    int putFile(const QString& localfile,const QString& remotefile);
//    int waitForSocket();
//    SftpSession session;

//    char                *buf;
//    int                  buflen;
//    long long            finish;
//    QFile                *file;
//};

//#endif // SFTP_H
