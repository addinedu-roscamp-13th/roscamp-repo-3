# Working Assumptions

## Network

- Operator/local IP: `192.168.2.8`
- JetCobot SSH target: `jetcobot@192.168.2.10`
- JetCobot SSH password: `1`

## Remote Workspace

- JetCobot project root:

```text
/home/jetcobot/wasab/roscamp-repo-3
```

## Usage Notes

- When syncing or checking JetCobot-side code, use:

```bash
sshpass -p 1 ssh jetcobot@192.168.2.10
```

- Remote Arm-related work should be done under:

```text
/home/jetcobot/wasab/roscamp-repo-3
```

- Previous JetCobot target information such as `192.168.0.33` is superseded by `192.168.2.10`.
