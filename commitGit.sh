# 自动提交所有子模块

commit_repo() {
    local repo_name=$1
    cd "src/$repo_name" || { echo "Repository $repo_name not found!"; return; }
    echo "Committing changes in $repo_name repository..."
    echo "Enter commit message, or press Enter to skip:"
    read message
    if [ -z "$message" ]; then
        echo "No commit message provided, skipping commit."
    else
        git add .
        git commit -m "$message"
    fi
    git push
    cd - > /dev/null || return
}

commit_repo "plane_part"
commit_repo "plane_part_assemble_task"
commit_repo "plane_part_moveit_config"
commit_repo "plane_part_stewart_driver"

echo "Commiting changes in main repository..."
echo "Enter commit message, or press Enter to skip:"
read message
if [ -z "$message" ]; then
    echo "No commit message provided, skipping commit."
else
    git add .
    git commit -m "$message"
    git push
fi
