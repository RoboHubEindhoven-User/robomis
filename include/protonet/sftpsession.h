#ifndef SFTPSESSION_H
#define SFTPSESSION_H

#include <cstdlib>
//#include "qsftp_export.h"
#include <QString>
#include <libssh2.h>
#include <libssh2_sftp.h>
#include <QMutex>
#include <QtNetwork/QTcpSocket>
#include <memory>

class SftpSession
{
public:
    SftpSession();
    ~SftpSession();
    bool connect(const QString& host,  const QString& username,const QString& password,int port, int timeout = 5000);
    bool disconnect();
    bool auth(bool (*prompt_cb)(const char*, char*, size_t, bool));
    bool startSftp();
    int errorCode() const;
    QString getSshError() const;
    QString getSftpError() const;
    QMutex* getMutex() const { return mutex; }
    LIBSSH2_SFTP* getSftp() { return sftp; }
    void setSftp(LIBSSH2_SFTP* sftp) { this->sftp = sftp; }
    LIBSSH2_SESSION* getSessionSsh() { return session_ssh; }
    int waitForSocket();
    void getLibsshError();
    int getErrorNumber() { return error_num; }

private:
    bool authed;
    QMutex* mutex;
    LIBSSH2_SESSION     *session_ssh;
    LIBSSH2_SFTP        *sftp;
    std::unique_ptr<QTcpSocket> socket;
    std::string         server_ip;
    std::string                 error_msg;
    int error_num;
    int                  error_msg_buflen;
};

#endif // SFTPSESSION_H
