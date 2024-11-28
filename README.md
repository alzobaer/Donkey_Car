# Steps to Install and Enable OpenSSH Server on Windows 11

## 1. Install OpenSSH Server

- 1.1 Open Settings → Apps → Optional Features.
- 1.2 Scroll down to find Add an optional feature and click - View features.
- 1.3 In the search box, type OpenSSH Server.
- 1.4 Select OpenSSH Server and click Install.
- 1.5 Wait for the installation to complete.

## 2. Start the OpenSSH Server

- 2.1 Open PowerShell as Administrator.
- 2.2 Start the SSH service

```bash
Start-Service sshd
```
- 2.3 Set it to start automatically at boot

``` bash
Set-Service -Name sshd -StartupType 'Automatic'
```

## 3. Verify SSH Service

After starting the service, verify it is running:

``` bash
Get-Service sshd
```



