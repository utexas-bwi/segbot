#!/bin/sh
#this configures sendmail to use gmail's smtp with the account utexas.bwi@gmail.com
#note that this method prevents the need for storing the password of the file in plain text on the filesystem
#this only needs to be done when you'd like to change the configuration.
if ! [ $(id -u) = 0 ] ; then
  echo "You must be root to run this script. Please run with sudo."
  exit 1
fi 
if [ $# -eq 1 ] ; then
  echo "Once finished, please test with this command: echo \"Just testing my sendmail gmail relay\" | mail -s \"Sendmail gmail Relay\" your.personal.email@company.com"
fi

printf "Email: "
read EMAIL
stty -echo
printf "Password: "
read PASSWORD
stty echo
printf "\n"

sudo apt-get install sendmail mailutils sendmail-bin
cd /etc/mail
sudo make clean
sudo mkdir -m 700 /etc/mail/authinfo/
cd /etc/mail/authinfo/
echo "AuthInfo: \"U:root\" \"I:$EMAIL\" \"P:$PASSWORD\"" > gmail-auth
sudo makemap hash gmail-auth < gmail-auth
sudo rm gmail-auth
echo "divert(-1)dnl
divert(0)dnl
define(\`_USE_ETC_MAIL_')dnl
include(\`/usr/share/sendmail/cf/m4/cf.m4')dnl
VERSIONID(\`\$Id: sendmail.mc, v 8.14.4-2ubuntu2.1 2013-09-19 22:03:58 cowboy Exp $')
OSTYPE(\`debian')dnl
DOMAIN(\`debian-mta')dnl
undefine(\`confHOST_STATUS_DIRECTORY')dnl        #DAEMON_HOSTSTATS=
FEATURE(\`no_default_msa')dnl
DAEMON_OPTIONS(\`Family=inet,  Name=MTA-v4, Port=smtp, Addr=127.0.0.1')dnl
DAEMON_OPTIONS(\`Family=inet,  Name=MSP-v4, Port=submission, M=Ea, Addr=127.0.0.1')dnl
define(\`confPRIVACY_FLAGS',dnl
\`needmailhelo,needexpnhelo,needvrfyhelo,restrictqrun,restrictexpand,nobodyreturn,authwarnings')dnl
define(\`confCONNECTION_RATE_THROTTLE', \`15')dnl
define(\`confCONNECTION_RATE_WINDOW_SIZE',\`10m')dnl
FEATURE(\`use_cw_file')dnl
FEATURE(\`access_db', , \`skip')dnl
FEATURE(\`greet_pause', \`1000')dnl 1 seconds
FEATURE(\`delay_checks', \`friend', \`n')dnl
define(\`confBAD_RCPT_THROTTLE',\`3')dnl
FEATURE(\`conncontrol', \`nodelay', \`terminate')dnl
FEATURE(\`ratecontrol', \`nodelay', \`terminate')dnl
include(\`/etc/mail/m4/dialup.m4')dnl
include(\`/etc/mail/m4/provider.m4')dnl
MAILER_DEFINITIONS
define(\`SMART_HOST',\`[smtp.gmail.com]')dnl
define(\`RELAY_MAILER_ARGS', \`TCP \$h 587')dnl
define(\`ESMTP_MAILER_ARGS', \`TCP \$h 587')dnl
define(\`confAUTH_OPTIONS', \`A p')dnl
TRUST_AUTH_MECH(\`EXTERNAL DIGEST-MD5 CRAM-MD5 LOGIN PLAIN')dnl
define(\`confAUTH_MECHANISMS', \`EXTERNAL GSSAPI DIGEST-MD5 CRAM-MD5 LOGIN PLAIN')dnl
FEATURE(\`authinfo',\`hash -o /etc/mail/authinfo/gmail-auth.db')dnl
LOCAL_CONFIG
include(\`/etc/mail/m4/dialup.m4')dnl
include(\`/etc/mail/m4/provider.m4')dnl
MAILER_DEFINITIONS
MAILER(local)dnl
MAILER(smtp)dnl
LOCAL_CONFIG" > /etc/mail/sendmail.mc
sudo make -C /etc/mail
sudo /etc/init.d/sendmail reload

