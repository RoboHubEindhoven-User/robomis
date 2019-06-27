//#include <protonet/sftpsession.h>
//#include <string>
//#include <iostream>
//#include <QString>
//#include <QDebug>
//#include <QMutex>
//#include <QMutexLocker>
//#include <QByteArray>
//#include <QObject>
//#include <QtNetwork/QTcpSocket>


//SftpSession::SftpSession()
//    : authed(false),
//      mutex(new QMutex()),
//      sftp(nullptr),
//      error_msg_buflen(1024) {

//}

//SftpSession::~SftpSession() {
//}

//bool SftpSession::connect(const QString& host,  const QString& username,const QString& password, int port, int timeout) {
//  QMutexLocker locker(mutex);
//  QByteArray tmp_username = username.toLatin1(); // must
//  QByteArray tmp_password = password.toLatin1(); // must
//  int        result;
//  char       *userauthlist;

//  socket = std::unique_ptr<QTcpSocket>(new QTcpSocket());
//  socket->connectToHost(host, quint16(port));

//  if (socket->waitForConnected(timeout)){
//      qDebug("Error connecting to host %s", host.toLocal8Bit().constData());
//      return -1;
//  }

//  if(!(session_ssh = libssh2_session_init())) {
//      fprintf(stderr, "Could not initialize the SSH session!\n");
//      return -1;
//  }

//  result = libssh2_session_handshake(session_ssh, socket->socketDescriptor());

//  if (result){
//      getLibsshError();
//      qDebug()<<QObject::tr("Failure establishing SSH session %1").arg(error_msg.c_str());
//      return -1;
//  }
//  libssh2_session_set_blocking(session_ssh , 0);
//      /* check what authentication methods are available */
//      userauthlist = nullptr;

//      while((userauthlist = libssh2_userauth_list(session_ssh , tmp_username.data(), static_cast<uint16_t>(username.size())))==nullptr && waitForSocket() > 0);

//      if(nullptr == userauthlist || strstr(userauthlist, "password") == nullptr){
//          getLibsshError();
//          disconnect();
//          qDebug()<<QObject::tr("Authentication not by password %1").arg(error_msg.c_str());
//          return -1;
//      }

//      while((result = libssh2_userauth_password(session_ssh , tmp_username.data(), tmp_password.data())) == LIBSSH2_ERROR_EAGAIN && waitForSocket() > 0);

//      if(result){
//          getLibsshError();
//          disconnect();
//          qDebug()<<QObject::tr("Authentication by password failed %1").arg(error_msg.c_str());
//          return -1;
//      }
////  return ssh_connect(ssh) == SSH_OK;
//  return true;
//}

//bool SftpSession::disconnect(){
//    if(session_ssh){
//        libssh2_session_disconnect(session_ssh ,"Normal shutdown");
//        libssh2_session_free(session_ssh );
//        session_ssh = nullptr;
//        return true;
//    }
//    return false;
//}

//int SftpSession::waitForSocket(){
//    struct timeval timeout;
//    int result;
//    fd_set fd;
//    fd_set *writefd = nullptr;
//    fd_set *readfd = nullptr;
//    int dir;
//    timeout.tv_sec = 10;
//    timeout.tv_usec = 0;
//    FD_ZERO(&fd);
//    FD_SET(socket->socketDescriptor(), &fd);

//    /* now make sure we wait in the correct direction */
//    dir = libssh2_session_block_directions(session_ssh);

//    if(dir & LIBSSH2_SESSION_BLOCK_INBOUND)
//        readfd = &fd;
//    if(dir & LIBSSH2_SESSION_BLOCK_OUTBOUND)
//        writefd = &fd;
//    result = select(socket->socketDescriptor() + 1, readfd, writefd, nullptr, &timeout);

//    return result;
//}

//void SftpSession::getLibsshError()
//{
//    error_num = libssh2_session_last_errno(session_ssh);
//    char* errmsg = nullptr;
//    int errmsg_len = 0;
//    char *msg;
//    libssh2_session_last_error(session_ssh, &msg, &errmsg_len, 0);
//    error_msg = msg;
//    if (errmsg_len > 0 && errmsg)
//    {
//        strncpy(msg, errmsg, static_cast<size_t>(error_msg_buflen));
//    }
//}
