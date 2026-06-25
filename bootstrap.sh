#!/usr/bin/env bash
# One-command bootstrap for the Jetson: install Ansible, fetch this repo, and
# converge the box locally. Safe to re-run (idempotent).
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/smyrgl/uav_ansible.git}"
BRANCH="${BRANCH:-main}"
CLONE_DIR="${CLONE_DIR:-$HOME/uav_ansible}"

echo ">> Installing Ansible + git ..."
sudo apt-get update
sudo apt-get install -y software-properties-common git
sudo add-apt-repository -y --update ppa:ansible/ansible || true
sudo apt-get install -y ansible

echo ">> Fetching playbook ($BRANCH) ..."
if [ -d "$CLONE_DIR/.git" ]; then
  git -C "$CLONE_DIR" fetch --depth 1 origin "$BRANCH"
  git -C "$CLONE_DIR" checkout "$BRANCH"
  git -C "$CLONE_DIR" reset --hard "origin/$BRANCH"
else
  git clone --depth 1 -b "$BRANCH" "$REPO_URL" "$CLONE_DIR"
fi
cd "$CLONE_DIR"

echo ">> Installing Galaxy collections ..."
ansible-galaxy collection install -r requirements.yml

echo ">> Converging ..."
# Add --ask-vault-pass once group_vars/vault.yml is encrypted.
sudo ansible-playbook -i inventory/localhost.yml site.yml "$@"

echo ">> Done. Some roles (kernel_modules/device_tree) may need a reboot;"
echo ">> reboot and re-run this script to finish."
