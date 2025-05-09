# 自动提交所有子模块

commit_repo() {
    local repo_name=$1
    cd "src/$repo_name" || { echo -e "\033[31mRepository $repo_name not found!\033[0m"; return; }
    echo -e "\033[34mCommitting changes in $repo_name repository...\033[0m"
    git status
    echo -e "\033[33mEnter commit message, or press Enter to skip:\033[0m"
    read message
    if [ -z "$message" ]; then
        echo -e "\033[31mNo commit message provided, skipping commit.\033[0m"
    else
        git add .
        git commit -m "$message"
        echo -e "\033[32mCommit successful!\033[0m"
    fi
    git push
    cd - > /dev/null || return
}

commit_repo "plane_part"
commit_repo "plane_part_assemble_task"
commit_repo "plane_part_moveit_config"
commit_repo "plane_part_stewart_driver"

echo -e "\033[34mCommiting changes in main repository...\033[0m"
git status
echo -e "\033[33mEnter commit message, or press Enter to skip:\033[0m"
read message
if [ -z "$message" ]; then
    echo -e "\033[31mNo commit message provided, skipping commit.\033[0m"
else
    git add .
    git commit -m "$message"
    git push
    echo -e "\033[32mCommit successful!\033[0m"
fi
