function navigateToLab(labPage) {
    window.location.href = labPage;
}

document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.collapsible').forEach(item => {
        // Create an arrow span
        const arrow = document.createElement('span');
        arrow.classList.add('collapsible-arrow');
        arrow.innerHTML = '▶'; // Right-pointing triangle
        item.insertBefore(arrow, item.firstChild);

        item.addEventListener('click', () => {
            const content = item.nextElementSibling;
            if (content.classList.contains('collapsible-content')) {
                const isExpanded = content.style.display === 'block';
                content.style.display = isExpanded ? 'none' : 'block';

                // Rotate arrow
                arrow.style.transform = isExpanded ? 'rotate(0deg)' : 'rotate(90deg)';
            }
        });
    });
});
