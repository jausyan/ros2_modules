# Contributing to ROS 2 Basic Tutorial

Terima kasih atas minat Anda untuk berkontribusi! 🎉

## Cara Berkontribusi

### Melaporkan Bug atau Kesalahan

Jika Anda menemukan kesalahan dalam tutorial:

1. Check apakah sudah ada issue yang sama
2. Buat issue baru dengan detail:
   - Deskripsi kesalahan
   - Langkah untuk reproduce
   - Environment (OS, ROS version)
   - Screenshot jika memungkinkan

### Memperbaiki Typo atau Kesalahan Kecil

1. Fork repository
2. Buat branch baru: `git checkout -b fix/typo-description`
3. Lakukan perubahan
4. Commit: `git commit -m "Fix typo in 01_pengenalan_ros2.md"`
5. Push: `git push origin fix/typo-description`
6. Buat Pull Request

### Menambah Konten atau Tutorial Baru

1. Diskusikan ide Anda di Issues terlebih dahulu
2. Fork dan buat branch: `git checkout -b feature/new-tutorial`
3. Tulis konten mengikuti format yang ada
4. Pastikan contoh code bisa dijalankan
5. Update README.md jika perlu
6. Buat Pull Request dengan deskripsi lengkap

### Memperbaiki Code Examples

1. Test code example terlebih dahulu
2. Pastikan mengikuti ROS 2 best practices
3. Tambahkan komentar yang jelas
4. Update dokumentasi terkait

## Style Guide

### Dokumentasi

- Gunakan Bahasa Indonesia yang jelas dan mudah dipahami
- Hindari jargon yang terlalu teknis tanpa penjelasan
- Sertakan contoh praktis
- Gunakan formatting Markdown yang konsisten

### Code

**Python:**
- Follow PEP 8
- Use descriptive variable names
- Add docstrings untuk functions/classes
- Sertakan error handling

**Example:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):
    """
    Brief description of node
    """
    
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node started!')
```

### Commit Messages

Format:
```
<type>: <subject>

<body>
```

Types:
- `feat`: Fitur baru
- `fix`: Bug fix
- `docs`: Dokumentasi
- `style`: Formatting
- `refactor`: Refactoring code
- `test`: Tests
- `chore`: Maintenance

Example:
```
docs: Add section about QoS in publisher tutorial

- Explain RELIABLE vs BEST_EFFORT
- Add code examples
- Update table of contents
```

## Code of Conduct

- Bersikap ramah dan profesional
- Hormati pendapat orang lain
- Fokus pada pembelajaran
- Bantu sesama learner

## Review Process

1. Maintainer akan review Pull Request Anda
2. Mungkin ada request untuk perubahan
3. Setelah approved, akan di-merge

## Testing

Sebelum submit PR:

```bash
# Test Python code
python3 -m flake8 src/
python3 -m pytest tests/

# Test ROS 2 package
cd ~/ros2_ws
colcon build --packages-select your_package
colcon test --packages-select your_package
```

## Questions?

Jika ada pertanyaan:
- Buat issue dengan label "question"
- Diskusi di Discussions (jika tersedia)

---

**Terima kasih atas kontribusi Anda! Setiap contribution, sekecil apapun, sangat dihargai! 🙏**
