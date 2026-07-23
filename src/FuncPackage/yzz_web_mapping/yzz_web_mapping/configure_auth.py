"""Create a private password-hash configuration for the web console."""
import argparse
import base64
import getpass
import hashlib
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='Configure YZZ web console login')
    parser.add_argument('--username', default='admin')
    parser.add_argument('--auth-file', default=str(Path.home() / '.config' / 'yzz_web_mapping' / 'auth.json'))
    args = parser.parse_args()
    password = os.environ.pop('YZZ_WEB_MAPPING_PASSWORD', '') or getpass.getpass('Web console password: ')
    if len(password) < 6:
        raise SystemExit('Password must contain at least 6 characters')
    salt = os.urandom(16)
    digest = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt, 200000)
    target = Path(args.auth_file).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(target.parent, 0o700)
    target.write_text(json.dumps({
        'username': args.username,
        'salt': base64.b64encode(salt).decode('ascii'),
        'password_hash': base64.b64encode(digest).decode('ascii'),
    }) + '\n', encoding='utf-8')
    os.chmod(target, 0o600)
    print(f'Login configuration written to {target}')


if __name__ == '__main__':
    main()
